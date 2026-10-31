"""Log and monitor Raspberry Pi 5 power while the pupper runs.

Run in its own terminal before starting lab_4:
    python3 power_monitor.py            # 10 Hz
    python3 power_monitor.py --rate 20
    python3 power_monitor.py --fast     # ~350 Hz, 5V rail only, to catch short dips

Every sample is written to power_logs/power_<timestamp>.csv and fsync'd,
so the log survives the Pi browning out and shutting down.
"""
import argparse
import csv
import os
import subprocess
import time
from datetime import datetime

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'power_logs')
UNDERVOLT_ALARM = '/sys/class/hwmon/hwmon3/in0_lcrit_alarm'  # rpi_volt
CPU_TEMP = '/sys/class/hwmon/hwmon0/temp1_input'              # cpu_thermal
WARN_VOLTS = 4.75  # Pi 5 wants 5.1V, flags undervoltage around 4.63V

# vcgencmd get_throttled bits
THROTTLE_FLAGS = {
    0: 'UNDERVOLT_NOW',
    1: 'FREQ_CAPPED_NOW',
    2: 'THROTTLED_NOW',
    3: 'TEMP_LIMIT_NOW',
    16: 'undervolt_since_boot',
    17: 'freq_capped_since_boot',
    18: 'throttled_since_boot',
    19: 'temp_limit_since_boot',
}


def vcgencmd(*args):
    return subprocess.run(['vcgencmd', *args], capture_output=True, text=True).stdout


def read_pmic():
    # lines look like: "     EXT5V_V volt(24)=4.81194000V"
    readings = {}
    for line in vcgencmd('pmic_read_adc').splitlines():
        name, _, value = line.strip().partition(' ')
        if '=' in value:
            readings[name] = float(value.split('=')[1].rstrip('VA'))
    return readings


def read_throttled():
    return int(vcgencmd('get_throttled').strip().split('=')[1], 16)


def read_file(path, default=-1):
    try:
        with open(path) as f:
            return int(f.read().strip())
    except OSError:
        return default


def decode_throttled(value):
    return [name for bit, name in THROTTLE_FLAGS.items() if value & (1 << bit)]


def fast_main():
    """Sample only the 5V rail as fast as vcgencmd allows (~350 Hz).

    Rows are buffered and written (fsync'd) every FLUSH_S, so up to that much
    is lost if the Pi dies. Prints each sample below WARN_VOLTS.
    """
    FLUSH_S = 0.25
    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f'power_fast_{datetime.now():%Y_%m_%d-%H_%M_%S}.csv')
    print(f'Fast logging to {log_path} (Ctrl+C to stop)')

    min_volts = float('inf')
    low_samples = 0
    count = 0
    start = time.monotonic()
    pending = []
    last_flush = start

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'elapsed_s', 'ext5v_v', 'undervolt_alarm'])

        def flush():
            writer.writerows(pending)
            pending.clear()
            f.flush()
            os.fsync(f.fileno())

        try:
            while True:
                t0 = time.monotonic()
                out = vcgencmd('pmic_read_adc', 'EXT5V_V')
                try:
                    volts = float(out.split('=')[1].strip().rstrip('V'))
                except (IndexError, ValueError):
                    continue
                alarm = read_file(UNDERVOLT_ALARM)
                pending.append([datetime.now().isoformat(timespec='milliseconds'),
                                f'{t0 - start:.3f}', f'{volts:.3f}', alarm])
                count += 1
                min_volts = min(min_volts, volts)
                if volts < WARN_VOLTS:
                    low_samples += 1
                    print(f'{t0 - start:8.3f}s  5V = {volts:.3f}V  alarm={alarm}')
                if t0 - last_flush >= FLUSH_S:
                    flush()
                    last_flush = t0
        except KeyboardInterrupt:
            flush()
            print(f'\nStopped. {count} samples ({count / (time.monotonic() - start):.0f} Hz), '
                  f'min 5V: {min_volts:.3f}V, samples below {WARN_VOLTS}V: {low_samples}')
            print(f'Log saved to {log_path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rate', type=float, default=10.0, help='samples per second')
    parser.add_argument('--fast', action='store_true',
                        help='5V rail only at max speed (~350 Hz) to catch short dips')
    args = parser.parse_args()
    if args.fast:
        return fast_main()
    period = 1.0 / args.rate

    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, f'power_{datetime.now():%Y_%m_%d-%H_%M_%S}.csv')
    print(f'Logging to {log_path} at {args.rate:g} Hz (Ctrl+C to stop)')

    min_volts = float('inf')
    undervolt_events = 0
    prev_alarm = 0
    last_print = 0.0
    start = time.monotonic()

    with open(log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time', 'elapsed_s', 'ext5v_v', 'vdd_core_a', 'cpu_temp_c',
                         'undervolt_alarm', 'throttled_hex', 'throttled_flags'])
        try:
            while True:
                t0 = time.monotonic()
                pmic = read_pmic()
                volts = pmic.get('EXT5V_V', float('nan'))
                core_amps = pmic.get('VDD_CORE_A', float('nan'))
                throttled = read_throttled()
                alarm = read_file(UNDERVOLT_ALARM)
                temp_c = read_file(CPU_TEMP) / 1000.0
                flags = decode_throttled(throttled)

                writer.writerow([datetime.now().isoformat(timespec='milliseconds'),
                                 f'{t0 - start:.2f}', f'{volts:.3f}', f'{core_amps:.3f}',
                                 f'{temp_c:.1f}', alarm, hex(throttled), '|'.join(flags)])
                f.flush()
                os.fsync(f.fileno())

                min_volts = min(min_volts, volts)
                if alarm == 1 and prev_alarm != 1:
                    undervolt_events += 1
                    print(f'\n!!! UNDERVOLTAGE at {t0 - start:.1f}s: 5V rail {volts:.3f}V')
                prev_alarm = alarm

                if t0 - last_print >= 0.5:
                    warn = '  <-- LOW' if volts < WARN_VOLTS else ''
                    print(f'\r5V: {volts:.3f}V (min {min_volts:.3f})  core: {core_amps:.2f}A  '
                          f'temp: {temp_c:.1f}C  undervolt events: {undervolt_events}  '
                          f'throttled: {hex(throttled)} {",".join(flags)}{warn}    ',
                          end='', flush=True)
                    last_print = t0

                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        except KeyboardInterrupt:
            print(f'\nStopped. Min 5V: {min_volts:.3f}V, undervoltage events: {undervolt_events}')
            print(f'Log saved to {log_path}')


if __name__ == '__main__':
    main()

"""Record commanded vs actual joint angles and foot positions for all 4 legs.

Run in its own terminal while lab_4.py is walking (on the ground):
    python3 leg_recorder.py                 # 100 Hz, Ctrl+C to stop
    python3 leg_recorder.py --rate 200
    python3 leg_recorder.py --summarize leg_logs/<file>.csv

Every sample goes to leg_logs/legs_<timestamp>.csv. Foot positions come from the
same FK functions lab_4.py uses (body frame), once from the commanded joint angles
and once from the measured ones. On stop it prints a per-leg summary: how far the
real foot is from the commanded foot while the foot is on the ground, and motor effort.
"""
import argparse
import csv
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import lab_4

LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'leg_logs')
LEGS = ['FR', 'FL', 'BR', 'BL']
JOINTS = [
    'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
    'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3',
    'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3',
    'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3',
]
STANCE_Z = -0.135  # commanded foot at or below this z (m) counts as on the ground

# the FK methods in lab_4.InverseKinematics never touch self
FK = [lab_4.InverseKinematics.fr_leg_fk, lab_4.InverseKinematics.fl_leg_fk,
      lab_4.InverseKinematics.br_leg_fk, lab_4.InverseKinematics.bl_leg_fk]


def foot_positions(theta):
    return np.concatenate([FK[i](None, theta[3 * i:3 * i + 3]) for i in range(4)])


def columns():
    cols = ['time', 'elapsed_s']
    for prefix in ('cmd', 'pos', 'effort'):
        cols += [f'{prefix}_{j}' for j in JOINTS]
    for prefix in ('cmd_ee', 'pos_ee'):
        cols += [f'{prefix}_{leg}_{axis}' for leg in LEGS for axis in 'xyz']
    return cols


class Recorder(Node):
    def __init__(self, rate):
        super().__init__('leg_recorder')
        self.pos = None
        self.effort = None
        self.cmd = None
        self.create_subscription(JointState, 'joint_states', self.on_joint_state, 10)
        self.create_subscription(Float64MultiArray, '/forward_command_controller/commands',
                                 self.on_command, 10)

        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = os.path.join(LOG_DIR, f'legs_{datetime.now():%Y_%m_%d-%H_%M_%S}.csv')
        self.file = open(self.log_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(columns())
        self.start = time.monotonic()
        self.rows = 0
        self.create_timer(1.0 / rate, self.sample)

    def on_joint_state(self, msg):
        self.pos = np.array([msg.position[msg.name.index(j)] for j in JOINTS])
        if len(msg.effort) == len(msg.name):
            self.effort = np.array([msg.effort[msg.name.index(j)] for j in JOINTS])

    def on_command(self, msg):
        self.cmd = np.array(msg.data)

    def sample(self):
        if self.pos is None or self.cmd is None:
            return
        effort = self.effort if self.effort is not None else np.full(12, np.nan)
        row = [datetime.now().isoformat(timespec='milliseconds'),
               f'{time.monotonic() - self.start:.3f}']
        row += [f'{v:.4f}' for v in np.concatenate([self.cmd, self.pos, effort,
                                                    foot_positions(self.cmd),
                                                    foot_positions(self.pos)])]
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % 50 == 0:
            self.file.flush()

    def close(self):
        self.file.close()


def summarize(path):
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader)
        data = np.array([[float(v) for v in row[1:]] for row in reader])
    col = {name: i - 1 for i, name in enumerate(header) if i > 0}
    print(f'{path}: {len(data)} samples, {data[-1, col["elapsed_s"]]:.1f} s')
    print('leg | stance% | foot z err mm (real-cmd) | foot xy err mm | x range real/cmd mm '
          '| mean |effort| j1 j2 j3 Nm')
    for leg in LEGS:
        cmd = np.stack([data[:, col[f'cmd_ee_{leg}_{a}']] for a in 'xyz'], axis=1)
        pos = np.stack([data[:, col[f'pos_ee_{leg}_{a}']] for a in 'xyz'], axis=1)
        stance = cmd[:, 2] <= STANCE_Z
        if not stance.any():
            print(f'{leg} | never on the ground')
            continue
        err = (pos - cmd)[stance]
        x_real = np.ptp(pos[:, 0]) * 1000
        x_cmd = np.ptp(cmd[:, 0]) * 1000
        effort = np.abs(np.stack([data[:, col[f'effort_{j}']] for j in
                                  JOINTS[3 * LEGS.index(leg):3 * LEGS.index(leg) + 3]], axis=1))
        print(f'{leg} | {100 * stance.mean():5.1f}   | {1000 * err[:, 2].mean():+8.1f} '
              f'(std {1000 * err[:, 2].std():.1f}) | {1000 * np.linalg.norm(err[:, :2], axis=1).mean():8.1f} '
              f'| {x_real:6.0f} / {x_cmd:.0f} | {effort.mean(0).round(3)}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rate', type=float, default=100.0, help='samples per second')
    parser.add_argument('--summarize', metavar='CSV', help='print the summary of a saved log and exit')
    args = parser.parse_args()
    if args.summarize:
        return summarize(args.summarize)

    rclpy.init()
    recorder = Recorder(args.rate)
    print(f'Recording to {recorder.log_path} at {args.rate:g} Hz (Ctrl+C to stop)')
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        path = recorder.log_path
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'\nSaved {recorder.rows} samples')
    if recorder.rows:
        summarize(path)


if __name__ == '__main__':
    main()

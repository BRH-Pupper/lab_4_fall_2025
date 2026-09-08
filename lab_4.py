import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from enum import IntEnum
import numpy as np
np.set_printoptions(precision=3, suppress=True)

## VERY IMPORTANT this is the defined order of ee_triangle_positions, fk_functions 
## thus affects the order of target_joint_positions_cache, target_ee_cache ... 
class Leg(IntEnum):
    FRONT_RIGHT = 0
    FRONT_LEFT = 1
    BACK_RIGHT = 2
    BACK_LEFT = 3
    TOTAL = 4
## VERY IMPORTANT - THIS VARIABLE MUST BE SET AND DETERMINE WHAT LEG TO MOVE (USED FOR DEBUGGING)
DESIRED_LEG = Leg.FRONT_RIGHT

## SEE https://www.youtube.com/watch?v=IsxojXns5Jg
class Gait(IntEnum):
    TROTTING = 0
    WALKING = 1
    CANTER = 2
    GALLOP = 3
    TOTAL = 4

## VERY IMPORTANT - THIS VARIABLE MUST BE SET TO DETERMINE WHAT GAIT TO USE
DESIRED_GAIT = Gait.TROTTING

def rotation_x(angle):
    return np.array(
                [
                    [1, 0, 0, 0],
                    [0, np.cos(angle), -np.sin(angle), 0],
                    [0, np.sin(angle), np.cos(angle), 0],
                    [0, 0, 0, 1],
                ])

def rotation_y(angle):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
     #rotation about y axis
    return np.array(
                [
                    [np.cos(angle),0, np.sin(angle), 0],
                    [0, 1, 0, 0],
                    [-np.sin(angle),0, np.cos(angle), 0],
                    [0, 0, 0, 1],
                ])


def rotation_z(angle):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
     return np.array([     
                    [np.cos(angle), -np.sin(angle),0, 0],
                    [np.sin(angle), np.cos(angle),0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                    ])
    

def translation(x, y, z):
    ################################################################################################
    # TODO: [already done] paste lab 2 forward kinematics here
    ################################################################################################
    return np.array([
                    [1,0, 0, x],
                    [0, 1,0, y],
                    [0, 0, 1, z],
                    [0, 0, 0, 1],
                    ])

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        # set up subscriber to JointState topic and set up callback when msg arrives with queue depth=10
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning
        
        # set up ROS publisher to forward command controller with Float64Array with queue depth=10
        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )
        # initialize velocities, positions, and target positions of joints 
        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None
        ## WARNING: self.t is no longer defined and a counter is used to iterate through gait trajectory positions
        # initialize counter to keep track of idx of target joint and feet 
        self.counter = 0

        # Trotting gait positions(XYZ), already implemented
        touch_down_position = np.array([0.05, 0.0, -0.14])
        stand_position_1 = np.array([0.025, 0.0, -0.14])
        stand_position_2 = np.array([0.0, 0.0, -0.14])
        stand_position_3 = np.array([-0.025, 0.0, -0.14])
        liftoff_position = np.array([-0.05, 0.0, -0.14])
        mid_swing_position = np.array([0.0, 0.0, -0.05])

        # some aliasing to make assignemnt easier to read (note assignment is by reference in python)
        td = touch_down_position
        s1 = stand_position_1
        s2 = stand_position_2
        s3 = stand_position_3
        lo = liftoff_position
        ms = mid_swing_position

        # array to better organize the gaits for each foot
        gait_array = {
            Gait.TROTTING: 
            {
                # NOTE: TROTTING gait is diagonal so FR and BL are in phase, FL and BR are in phase
                Leg.FRONT_RIGHT: np.array([td, s1, s2, s3, lo, ms]),
                Leg.FRONT_LEFT: np.array([lo, ms, td, s1, s2, s3]),
                Leg.BACK_RIGHT: np.array([lo, ms, td, s1, s2, s3]),
                Leg.BACK_LEFT: np.array([td, s1, s2, s3, lo, ms])
            },
            # NOTE: these below are unimplemented 
            Gait.WALKING: 
            {
                Leg.FRONT_RIGHT: np.array([]),
                Leg.FRONT_LEFT: np.array([]),
                Leg.BACK_RIGHT: np.array([]),
                Leg.BACK_LEFT: np.array([])
            },
            Gait.CANTER:
            {
                Leg.FRONT_RIGHT: np.array([]),
                Leg.FRONT_LEFT: np.array([]),
                Leg.BACK_RIGHT: np.array([]),
                Leg.BACK_LEFT: np.array([])
            },
            Gait.GALLOP:
            {
                Leg.FRONT_RIGHT: np.array([]),
                Leg.FRONT_LEFT: np.array([]),
                Leg.BACK_RIGHT: np.array([]),
                Leg.BACK_LEFT: np.array([])
            }
        }
        
        ## trotting
        # Take positions defining gait and add offset of each foot to get trajectory  
        # note each leg iterates through different order of trotting gait positions
        rf_ee_offset = np.array([0.06, -0.09, 0]) # right front foot
        rf_ee_triangle_positions = gait_array[DESIRED_GAIT][Leg.FRONT_RIGHT] + rf_ee_offset
        
        lf_ee_offset = np.array([0.06, 0.09, 0]) # left front foot
        lf_ee_triangle_positions = gait_array[DESIRED_GAIT][Leg.FRONT_LEFT] + rf_ee_offset
        
        rb_ee_offset = np.array([-0.11, -0.09, 0]) # right back foot
        rb_ee_triangle_positions = gait_array[DESIRED_GAIT][Leg.BACK_RIGHT] + rb_ee_offset
        
        lb_ee_offset = np.array([-0.11, 0.09, 0]) # left back foot
        lb_ee_triangle_positions = gait_array[DESIRED_GAIT][Leg.BACK_LEFT] + lb_ee_offset

        # store triangle trajectory of each foot and forward kinematics functions for each leg in a list for easy access
        self.ee_triangle_positions = [rf_ee_triangle_positions, lf_ee_triangle_positions, rb_ee_triangle_positions, lb_ee_triangle_positions]
        self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.bl_leg_fk]
        # calculates IK ahead of time to get and cache target joint and position of each foot 
        # interpolates the joints and position between each position in the gait trajectory
        self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
        print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
        print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')

        # define period of timers for PD and IK calculations (defines how often we update the target joint positions)
        # TODO someone needs ot make decision to determine speed of gait by update rate (self) 
        # or defined gait period (i.e. self.ik_timer_period) or self.gate_period_cycle)
        self.pd_timer_period = 1.0 / 200  # 200 Hz = 5ms
        self.ik_timer_period = 1.0 / 100   #100 Hz = 10ms 
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)
        

    def fr_leg_fk(self, theta):
        # Already implemented in Lab 2
        T_RF_0_1 = translation(0.07500, -0.08350, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_RF_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_RF_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_RF_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_RF_0_ee = T_RF_0_1 @ T_RF_1_2 @ T_RF_2_3 @ T_RF_3_ee
        return T_RF_0_ee[:3, 3]

    def fl_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
                
        T_FL_0_1 = translation(0.07500, 0.08350, 0) @ rotation_x(-1.57080) @ rotation_z(theta[0])
        T_FL_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_FL_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_FL_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_FL_0_ee = T_FL_0_1 @ T_FL_1_2 @ T_FL_2_3 @ T_FL_3_ee
        return T_FL_0_ee[:3, 3]

    def br_leg_fk(self, theta):
        T_BR_0_1 = translation(-0.07500, -0.0725, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
        T_BR_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_BR_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_BR_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_BR_0_ee = T_BR_0_1 @ T_BR_1_2 @ T_BR_2_3 @ T_BR_3_ee
        return T_BR_0_ee[:3, 3]

    def bl_leg_fk(self, theta):
        ################################################################################################
        # TODO: implement forward kinematics here
        ################################################################################################
        T_BL_0_1 = translation(-0.07500, 0.0725, 0) @ rotation_x(-1.57080) @ rotation_z(theta[0])
        T_BL_1_2 = rotation_y(-1.57080) @ rotation_z(theta[1])
        T_BL_2_3 = translation(0, -0.04940, 0.06850) @ rotation_y(1.57080) @ rotation_z(theta[2])
        T_BL_3_ee = translation(0.06231, -0.06216, 0.01800)
        T_BL_0_ee = T_BL_0_1 @ T_BL_1_2 @ T_BL_2_3 @ T_BL_3_ee
        return T_BL_0_ee[:3, 3]

    def forward_kinematics(self, theta):
        return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])

    # callback to read msg to get position and velocities of joints of interest
    def listener_callback(self, msg):
        joints_of_interest = [
            'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3', 
            'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3', 
            'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3', 
            'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
        ]
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    # calculate IK - find desired foot position given angle 
    def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0, 0, 0]):
        leg_forward_kinematics = self.fk_functions[leg_index]

        def cost_function(theta):
            current_position = leg_forward_kinematics(theta)
            error = np.abs(target_ee - current_position)
            L2_norm = np.linalg.norm(error)
            cost = (L2_norm)**2
            return cost, L2_norm

        def gradient(theta, epsilon=1e-3):
            grad = np.zeros(len(theta))
            cost,_ = cost_function(theta)
            for i in range(len(theta)):
                nudged_theta = theta.copy()
                nudged_theta[i]= nudged_theta[i]+epsilon 
                nudged_cost,_ = cost_function(nudged_theta) 
                grad[i] = (nudged_cost - cost)/epsilon
            return grad

        theta = np.array(initial_guess)
        learning_rate = 5 
        max_iterations = 50 
        tolerance = 1e-3

        cost_l = []
        for _ in range(max_iterations):
            grad = gradient(theta)

            theta = theta - learning_rate* grad

            cost,_ = cost_function(theta)
            cost_l.append(cost)
            current_position = leg_forward_kinematics(theta)
            error = np.abs(target_ee - current_position)
            if (abs(error[0])+abs(error[1])+abs(error[2]))<tolerance:
                break

        #print(f'Cost: {cost_l}') # Use to debug to see if your cost function converges within max_iterations

        return theta

    # interpolate point between gait positions
    #                              Mid-Swing
    #                                 ○
    #                               /   \
    #                             /       \
    #                           /           \
    #                         /               \
    #                       /                   \
    #                     /                       \
    #                   /                           \
    #                 /                               \
    #               ○ ------ ○ ------ ○ ------ ○ ------ ○
    #           Lift-Off   Stand3  Stand2  Stand1   Touch Down
    #              ←         ←        ←      ←          ← 
    #
    #                       Walking direction: ←
    #
    # Gait cycle:
    # Touch Down -> Stand 1 -> Stand 2 -> Stand 3 -> Lift-Off
    #            -> Mid-Swing -> Touch Down -> ...

    def interpolate_triangle(self, t, leg_index):
        
        gait_positions = self.ee_triangle_positions[leg_index]
        num_gait_pos = len(gait_positions)
        gait_position_period = 1.0/num_gait_pos

        start_segment_idx = int(t/gait_position_period)
        next_segment_idx = (start_segment_idx + 1)%  num_gait_pos

        start_pos = gait_positions[start_segment_idx]
        next_pos = gait_positions[next_segment_idx]

        segment_start_time = start_segment_idx * gait_position_period
        segment_progress = (t - segment_start_time) / gait_position_period

        return start_pos + (next_pos - start_pos) * segment_progress
        # t = t%3
        # start = None
        # end = None
        # v = self.ee_triangle_positions[leg_index]
        # if t<1:
        #     start = v[0]
        #     end = v[1]
        # elif t<2:
        #     start = v[1]
        #     end = v[2]
        # else:
        #     start = v[2]
        #     end = v[0]
        # return start + (end - start)*(t%1)

    # precomputes walking cycle joints and positions
    def cache_target_joint_positions(self):
        # Calculate and store the target joint positions for a cycle and all 4 legs
        target_joint_positions_cache = []
        target_ee_cache = []
        for leg_index in range(Leg.TOTAL):
            target_joint_positions_cache.append([])
            target_ee_cache.append([])
            target_joint_positions = [0] * 3
            ## WARNING t is misnomer - doesn't really represent time anymore from lab 3
            # t defines PROGRESS of the GAIT or percentage i.e. t = 0.20 = 20% of gait trajectory
            ## TODO should be replaced with time to make more sense (EDSUN's opinion)
            
            TOTAL_GAIT_PROGRES = 1.0
            GAIT_PROGRESS_INCREMENT = 0.02
            for t in np.arange(0, TOTAL_GAIT_PROGRES, GAIT_PROGRESS_INCREMENT):
                print(t)
                # get expected position of foot in gait cycle
                target_ee = self.interpolate_triangle(t, leg_index)
                # calculate the joint to achieve that foot position using IK
                target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)

                target_joint_positions_cache[leg_index].append(target_joint_positions)
                target_ee_cache[leg_index].append(target_ee)

        # (4, 50, 3) -> (50, 12) - combing 4 arrays of 50x3 into one array of 50x12
        target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
        target_ee_cache = np.concatenate(target_ee_cache, axis=1)
        
        return target_joint_positions_cache, target_ee_cache

    def get_target_joint_positions(self):
        target_joint_positions = self.target_joint_positions_cache[self.counter]
        target_ee = self.target_ee_cache[self.counter]
        self.counter += 1
        if self.counter >= self.target_joint_positions_cache.shape[0]:
            self.counter = 0
        return target_ee, target_joint_positions

    # periodic callback to retrieve next frame from cache gait positions/cycle/target joint then calculate current foot position using FK
    # misnomer - doesn't calculate IK because calculated already in init and cached. Actually needed to advance the gait
    def ik_timer_callback(self):
        if self.joint_positions is not None:
            target_ee, target_joint_positions = self.get_target_joint_positions()
            current_ee = self.forward_kinematics(self.joint_positions)

            # FR = 0 -> start=0, end=3 -> [0:3]
            # FL = 1 -> start=3, end=6 -> [3:6]
            # BR = 2 -> start=6, end=9 -> [6:9]
            # BL = 3 -> start=9, end=12 -> [9:12]
            if DESIRED_LEG != Leg.TOTAL:
                start = DESIRED_LEG * 3
                end = (DESIRED_LEG + 1) * 3
                self.target_joint_positions = self.joint_positions.copy()
                desired_ee = current_ee.copy()

                self.target_joint_positions[start:end] = target_joint_positions[start:end]
                desired_ee[start:end] = target_ee[start:end]

                target_ee = desired_ee
            else:
                self.target_joint_positions = target_joint_positions

            self.get_logger().info(
                f'Target EE: {target_ee}, \
                Current EE: {current_ee}, \
                Target Angles: {self.target_joint_positions}, \
                Target Angles to EE: {self.forward_kinematics(self.target_joint_positions)}, \
                Current Angles: {self.joint_positions}')

    # callback to publish target joint positions to controller
    # misnomer - not actually calculating PD control, just sending msg
    def pd_timer_callback(self):
        if self.target_joint_positions is not None:
            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)

def main():
    # initializes ros2 python client library
    rclpy.init()
    # instaniates inverse kinematics node
    inverse_kinematics = InverseKinematics()
    
    try:
        # keep node alive and listening for messages until interrupted
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques to 4 legs * 3 joints = 12 torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0] * 12
        inverse_kinematics.command_publisher.publish(zero_torques)
        # destroy and clean ik node
        inverse_kinematics.destroy_node()
        # clean up ros2 associated resources to be cleaned up
        rclpy.shutdown()

if __name__ == '__main__':
    main()
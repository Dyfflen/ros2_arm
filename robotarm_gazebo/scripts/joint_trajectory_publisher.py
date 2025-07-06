#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header

arm_joints = [ 'base_link_to_link1', 
               'link1_to_link2', 
               'link2_to_link3', 
               'link3_to_link4', 
               'link4_to_gripper_base']

gripper_joints = ['gripper_controller']

class ExampleJointTrajectoryPublisherPy(Node):
    def __init__(self):
        super().__init__('example_joint_trajectory_publisher_py')    
        self.arm_pose_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.gripper_pose_publisher = self.create_publisher(JointTrajectory, '/grip_controller/joint_trajectory', 1)

        self.timer_period = 5.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.frame_id = "base_link"

        self.duration_sec = 2
        self.duration_nanosec = 0.5 * 1e9

        self.arm_positions = []
        self.arm_positions.append([0.0, 0.0, 0.0, 0.0, 0.0]) # Home location
        self.arm_positions.append([-1.292, 0.856, 0.452, -2.350, -2.008]) # Goal location
        self.arm_positions.append([-1.292, 0.856, 0.452, -2.350, -2.008])
        self.arm_positions.append([0.0, 0.0, 0.0, 0.0, 0.0]) # Home location

        self.gripper_positions = []
        self.gripper_positions.append([-0.700]) # Open gripper
        self.gripper_positions.append([-0.700])
        self.gripper_positions.append([0.150]) # Close gripper
        self.gripper_positions.append([0.150]) 
        self.index = 0

    def timer_callback(self):
        msg_arm = JointTrajectory()
        msg_arm.header = Header()  
        msg_arm.header.frame_id = self.frame_id  
        msg_arm.joint_names = arm_joints

        msg_gripper = JointTrajectory()
        msg_gripper.header = Header()  
        msg_gripper.header.frame_id = self.frame_id  
        msg_gripper.joint_names = gripper_joints

        point_arm = JointTrajectoryPoint()
        point_arm.positions = self.arm_positions[self.index]
        point_arm.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))
        msg_arm.points.append(point_arm)
        self.arm_pose_publisher.publish(msg_arm)

        point_gripper = JointTrajectoryPoint()
        point_gripper.positions = self.gripper_positions[self.index]
        point_gripper.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))  
        msg_gripper.points.append(point_gripper)
        self.gripper_pose_publisher.publish(msg_gripper)

        # Reset the index
        if self.index == len(self.arm_positions) - 1:
            self.index = 0
        else:
            self.index = self.index + 1
    
def main(args=None):
    rclpy.init(args=args)
    example_joint_trajectory_publisher_py = ExampleJointTrajectoryPublisherPy()
    rclpy.spin(example_joint_trajectory_publisher_py)
    example_joint_trajectory_publisher_py.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()

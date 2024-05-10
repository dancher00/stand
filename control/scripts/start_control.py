#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from utils.kinematics import Kinematics
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class Controller:
    def __init__(self) -> None:
        self.kin = Kinematics()
        self.current_angles = np.array([0, 0, 0])

    def start_plan(self):
        self.jt = JointTrajectory()
        self.jt.joint_names = ["Revolute_4", "Revolute_6", "Revolute_8"]

    def set_current_angles(self, angles):
        self.current_angles = angles

    def move_to_position(self, pose, dt):
        angles, state_pred = self.kin.ik(pose, self.current_angles)
        # state = self.kin.fk(self.current_angles)[-1]
        self.jt.points.append(
            JointTrajectoryPoint(
                positions=angles,
                time_from_start=rospy.Duration(dt),
            )
        )


class RosManager:
    def __init__(self) -> None:
        rospy.init_node("listener", anonymous=True)
        self.controller = Controller()
        rospy.Subscriber("/leg/joint_states", JointState, self.listen_joint_state)
        self.planer = Planer()

    def spin(self):
        rate = rospy.Rate(10)
        pub = rospy.Publisher(
            "leg/joint_trajectory_controller/command", JointTrajectory, queue_size=10
        )
        while not rospy.is_shutdown():
            trajectory = self.plan()
            pub.publish(trajectory)
            rate.sleep()

    def listen_joint_state(self, msg: JointState):
        angles = np.array(msg.position[:-1])
        self.controller.set_current_angles(angles)

    def plan(self):
        next_xyz, execution_time = self.planer.step()
        if execution_time < 0:
            print("wrong execution_time")
            print(execution_time)
        self.controller.start_plan()
        self.controller.move_to_position(next_xyz, execution_time)
        trajectory = self.controller.jt
        return trajectory


class Planer:
    def __init__(self) -> None:
        self.stages = rospy.get_param("/planer/stages")
        self.current_stage = self.stages[0]
        self.dt_stages = {
            stage: rospy.get_param("/planer/stage_duration/" + stage)
            for stage in self.stages
        }
        self.stage_pos = {
            stage: rospy.get_param("/planer/stage_position/" + stage)
            for stage in self.stages
        }
        self.stage_order = {
            stage: rospy.get_param("/planer/stage_order/" + stage)
            for stage in self.stages
        }
        self.start_time = rospy.Time.now()

    def step(self):
        execution_time = (self.start_time - rospy.Time.now()).to_sec() + self.dt_stages[
            self.current_stage
        ]
        if execution_time > 0:
            return (self.stage_pos[self.current_stage], execution_time)
        else:
            self.start_time = rospy.Time.now()
            self.current_stage = self.stage_order[self.current_stage]
            return self.step()


if __name__ == "__main__":
    ros_manager = RosManager()
    ros_manager.spin()

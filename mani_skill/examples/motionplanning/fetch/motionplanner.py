import mplib
import numpy as np
from mani_skill.examples.motionplanning.panda.motionplanner import PandaArmMotionPlanningSolver

class FetchArmMotionPlanningSolver(PandaArmMotionPlanningSolver):
    def setup_planner(self):
        ignore_links = ["r_wheel_link","l_wheel_link","estop_link","laser_link"]
        link_names = []
        for link in self.robot.get_links():
            if link.get_name() not in ignore_links:
                link_names.append(link.get_name())
        #link_names = [link.get_name()  if link not in ignore_link]
        joint_names = [joint.get_name() for joint in self.robot.get_active_joints()]
        print("joint_names", joint_names, len(joint_names))
        planner = mplib.Planner(
            urdf=self.env_agent.urdf_path.replace(".urdf","_mplib.urdf"),
            srdf=self.env_agent.urdf_path.replace(".urdf", ".srdf"),
            user_link_names=link_names,
            user_joint_names=joint_names,
            move_group="gripper_link",
            joint_vel_limits=np.ones(11) * self.joint_vel_limits,
            joint_acc_limits=np.ones(11) * self.joint_acc_limits,
        )
        planner.set_base_pose(np.hstack([self.base_pose.p, self.base_pose.q]))
        return planner


import rospy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

class CranePlanningDataGenerator(object):
    def __init__(self):
        super(CranePlanningDataGenerator, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('crane_planningdata_generator', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "whole"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", self.robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print ""

        print self.move_group.get_current_joint_values()

    def plan(self, init_conf, goal_conf, planner_id):
        RobotState
        JointState
        init_state = self.robot.get_current_state()
        init_state.joint_state.position = init_conf
        self.move_group.set_start_state(init_state)
        goal_joint = self.move_group.get_current_joint_values()
        goal_joint = goal_conf


def main():
    gen = CranePlanningDataGenerator()

if __name__ == '__main__':
    main()



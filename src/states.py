# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL
f = open("/home/aditya/ws_moveit/src/motion_planner/file.csv", "w")

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        # robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        # scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        # display_trajectory_publisher = rospy.Publisher(
            # "/move_group/display_planned_path",
            # moveit_msgs.msg.DisplayTrajectory,
            # queue_size=20,
        # )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        # planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)
        #
        # # We can also print the name of the end-effector link for this group:
        # eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)
        #
        # # We can get a list of all the groups in the robot:
        # group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())
        #
        # # Sometimes for debugging it is useful to print the entire state of the
        # # robot:
        # print("============ Printing robot state")
        # print(move_group.get_current_pose())
        # time = move_group.get_current_pose().header.stamp.secs + move_group.get_current_pose().header.stamp.nsecs * 1e-9
        # string = str(time) +  ", "  +str(move_group.get_current_pose().pose.position.x) + ", " +  str(move_group.get_current_pose().pose.position.y) + ", " + str(move_group.get_current_pose().pose.position.z) + "\n"
        # f.write(string)
        # # print("")
        # ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        # self.robot = robot
        # self.scene = scene
        self.move_group = move_group
        # self.display_trajectory_publisher = display_trajectory_publisher

def main():
    
    rospy.init_node('new', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    
    # f.open("/home/aditya/ws_moveit/src/motion_planner/file.csv", "a")
    f.write("Time, X, Y, Z \n")
    tutorial = MoveGroupPythonInterfaceTutorial()
    
    while not rospy.is_shutdown():
                
        time = tutorial.move_group.get_current_pose().header.stamp.secs + tutorial.move_group.get_current_pose().header.stamp.nsecs * 1e-9
        string = str(time) +  ", "  +str(tutorial.move_group.get_current_pose().pose.position.x) + ", " +  str(tutorial.move_group.get_current_pose().pose.position.y) + ", " + str(tutorial.move_group.get_current_pose().pose.position.z) + "\n"
        f.write(string)
        print(string)
        rate.sleep()

    print("finished")
    f.close()

if __name__ == "__main__":
    main()

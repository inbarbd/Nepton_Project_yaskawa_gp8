#!/usr/bin/env python
from __future__ import print_function
from __future__ import division
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
import itertools
import math
import numpy as np
import time
import pickle
import copy
# NeptonStartPose =  [-0.007537535885248303, 0.36035792391667876, 0.8599901204581295, -0.00211895775982196, -1.9293202283370317, 0.008573520641837362]
NeptonStartPose = [-1.4972114723300667, 0.30999879612380266, -0.017981686586479917, -0.0024347935789119646, -0.8608923740912839, 0.006483615911854201]
"""
Max Amplitud 0.27
"""
# T = 10
# frequency = 1/T
# movment_time_s = 60
# JOINT_CONTROLLER = '/joint_command' #high rate control
JOINT_CONTROLLER = '/joint_path_command' #low rate control
JOINT_STAT_TOPIC = "/joint_states"
# JOINT_CONTROLLER = '/motoman_gp8/gp8_controller/command' #Gazebo control
# JOINT_STAT_TOPIC = "/motoman_gp8/joint_states"

Const = 0.4

class gp8_real_time_position_control:
    def __init__(self, rate=1):
        self.rate = rate
        self.duration = 1
        self.step = 1
        self.time_from_start = 0
        self.point = None
        self.old_point = None
        # print(frequency,"frequency")

        self.cycle_time = 60

        self.max_speed_factor = 1.0  # % of max speed for safety reasons
        self.max_speed = np.array([400, 385, 520, 550, 550, 1000])*np.pi/180  # rad/s
        # print(self.max_speed,"max_speed")
        joint_states_sub = rospy.Subscriber(JOINT_STAT_TOPIC, JointState, self.joint_states_callback)
        self.joint_names = None
        self.joint_angles = None
        self.joint_velocity = None

        joints_data = None
        while joints_data is None:
            try:
                joints_data = rospy.wait_for_message(JOINT_STAT_TOPIC, JointState, timeout=5)
            except:
                rospy.logwarn("Time out ")
                pass

        self.publish_topic = rospy.Publisher(JOINT_CONTROLLER , JointTrajectory, queue_size=1)
        self.rate_ros = rospy.Rate(self.rate)

        while self.publish_topic.get_num_connections() == 0:
            self.rate_ros.sleep()
        # print("Established Connection")

    def joint_states_callback(self, msg):
        self.joint_names = msg.name
        self.joint_angles = msg.position
        self.joint_velocity = msg.velocity

    def move_to_joint(self, path):
        
        self.path = path
        return self.joint_trajectory_msg()

    def joint_trajectory_msg(self):
        self.time_index = 0
        joint_traj = JointTrajectory()

        joint_traj.joint_names = self.joint_names
        joint_traj.header.frame_id = ''
        joint_traj.header.seq = 0
        
        # points = [self.joint_trajectory_point(dt)]
        points = []
        for t in range(0,len(self.path)+1):
            if t == 0:
                NextPoint = JointTrajectoryPoint()
                NextPoint.positions = self.joint_angles
                NextPoint.velocities = [0]*len(self.joint_angles)
                NextPoint.time_from_start = rospy.Duration(0.0)
                points.append(NextPoint)
            else:
                # print(self.dt*t,"self.dt*t")
                points.append(self.joint_trajectory_point(self.path[t-1],self.dt*t))
                self.time_index = self.time_index + 1
        joint_traj.points = points
        # print(len(points))
        return joint_traj

    def joint_trajectory_point(self,PosList, time_from_start):
        # print(len(self.path))
        NextPoint = JointTrajectoryPoint()
        NextPoint.positions = PosList
        NextPoint.velocities = self.calculate_joint_velocity(time_from_start)
        NextPoint.time_from_start = rospy.Duration(time_from_start)
        return NextPoint

    def calculate_joint_velocity(self, time):
        velocity_vector = []      
        if time == self.dt :
            pos = self.joint_angles
            # print(self.path)
            for joint in range(0,len(self.joint_angles)):
                joint_velocity = self.path[self.time_index-1][joint] - pos[joint]
                joint_velocity = joint_velocity/self.dt
                velocity_vector.append(joint_velocity)
        if time > self.dt:
            for joint in range(0,len(self.joint_angles)):
                joint_velocity = self.path[self.time_index-1][joint] - self.path[self.time_index-2][joint]
                joint_velocity = joint_velocity/self.dt
                velocity_vector.append(joint_velocity)
        # print("velocety:", velocity_vector)
        return velocity_vector

    def publish(self, trajectory):
        self.publish_topic.publish(trajectory)
        time.sleep(1.0 / self.rate)
        self.old_point = self.point

    @staticmethod
    def go_to_start(self):
        # Publish current position at first
        trajectory = self.move_to_joint([self.joint_angles,START_POS])
        self.publish(trajectory)
        time.sleep(1)
        print("Ready")

    def follow_costum_path(self,path):
        self.cycle_time = rospy.get_param('cycle_time')
        self.dt = self.cycle_time/2
        trajectory = self.move_to_joint(path)
        print(trajectory,"trajectory")
        self.publish(trajectory)

    def follow_ashkelon_path(self,path,file_dic):
        self.time_index = 0
        joint_traj = JointTrajectory()
        joint_traj.joint_names = self.joint_names
        joint_traj.header.frame_id = ''
        joint_traj.header.seq = 0        
        # points = [self.joint_trajectory_point(dt)]
        points = []
        time_from_start = 0
        for t in range(0,len(path)+1):
            if t == 0:
                NextPoint = JointTrajectoryPoint()
                NextPoint.positions = self.joint_angles
                NextPoint.velocities = [0]*len(self.joint_angles)
                NextPoint.time_from_start = rospy.Duration(0.0)
                points.append(NextPoint)
            else:
                time_from_start = time_from_start + file_dic[t-1]["duration"]
                NextPoint = JointTrajectoryPoint()
                NextPoint.positions = path[t-1]
                velocity_vector = []
                for joint in range(0,len(path[t-1])):
                    if t == 1:
                        joint_velocity = path[t-1][joint] - self.joint_angles[joint]
                    if t > 1:
                        joint_velocity = path[t-1][joint] - path[t-2][joint]
                    joint_velocity = joint_velocity/file_dic[t-1]["duration"]
                    velocity_vector.append(joint_velocity)
                NextPoint.velocities = velocity_vector
                NextPoint.time_from_start = rospy.Duration(time_from_start)
                points.append(NextPoint)
                # self.time_index = self.time_index + 1
            
        joint_traj.points = points
        # print(len(points))
        # print(joint_traj,"joint_traj")
        self.publish(joint_traj)

class MoveGroupPythonInterface(object):
    """MoveGroupPythonInterfacemoveit_class"""

    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "nepton_gp8"
        move_group = moveit_commander.MoveGroupCommander(group_name)
  
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)
        group_names = robot.get_group_names()
        # print(robot.get_current_state())
        # print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        # self.add_obstacles()

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        # joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self,pose):
        move_group = self.move_group
        pose_goal = pose

        move_group.set_pose_target(pose_goal)
        success = move_group.go(wait=True)
        print(success,"success")
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        return success

    def plan_cartesian_path(self, scale=1):
        """ Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified
         as waypoints. Configurations are computed for every eef_step meters;
         The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath.
         The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. """

        move_group = self.move_group
        waypoints = []

        signs = [1, -1]
        # Create an iterator that cycles through the list of signs indefinitely
        sign_cycle = itertools.cycle(signs)

        wpose = move_group.get_current_pose().pose
        start_pose = copy.deepcopy(wpose)
        for t in range(movment_time):
            sign = next(sign_cycle)  
            wpose.position.z = start_pose.position.z + Amplitude*math.sin(sign*math.pi/2)  # First move up (z)
            print(start_pose.position.z,"start_pose.position.z")
            print(wpose.position.z)
            # wpose.time_from_start.secs = T/2
            waypoints.append(copy.deepcopy(wpose))
        print(len(waypoints),"len(waypoints)")
        # wpose.position.z -= scale * 0.1  # First move up (z)
        # wpose.position.y += scale * 0.2  # and sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))

        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints ,0.01 , 0.00 ) # waypoints to follow  # eef_step # jump_threshold
         
        print(fraction,"fraction")
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def get_cartesian_pose(self):
        arm_group = self.move_group

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

    def go_start_pose(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = NeptonStartPose[0]
        joint_goal[1] = NeptonStartPose[1]
        joint_goal[2] = NeptonStartPose[2]
        joint_goal[3] = NeptonStartPose[3]
        joint_goal[4] = NeptonStartPose[4]
        joint_goal[5] = NeptonStartPose[5]
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()

    @staticmethod
    def all_close(goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats
        @param: actual     A list of floats
        @param: tolerance  A list of floats
        @returns: bool
        """
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance[index]:
                if index > 2:  # for angles
                    if abs(actual[index] - goal[index]) < 2*pi - tolerance[index]:  # 2 pi with tolerance
                        return False
                else:
                    return False
        return True

    def add_obstacles(self, timeout=4):
        print("add_obstacles")
        floor = {'name': 'floor', 'pose': [0, 0, -0.01], 'size': (3, 3, 0.02)}
        # Adding Objects to the Planning Scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = floor['pose'][0]
        box_pose.pose.position.y = floor['pose'][1]
        box_pose.pose.position.z = floor['pose'][2]
        self.box_name = floor['name']
        self.scene.add_box(self.box_name, box_pose, size=floor['size'])
        self.scene.attach_box('base_link', self.box_name)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Ensuring Collision Updates Are Receieved
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are made, we wait until we see the
        # changes reflected in the ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        start = time.time()  # rospy.get_time()
        seconds = time.time()  # rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene. -Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            time.sleep(0.1)
            seconds = time.time()  # rospy.get_time()
        # If we exited the while loop without returning then we timed out
        return False

class nepton_program(object):
    def __init__(self):
        
        self.moveit_class = MoveGroupPythonInterface()
        self.position_manipulator = gp8_real_time_position_control()
        self.Amplitude = 0

    def test_max_limit(self,req):
        Amplitude = rospy.get_param('amplitude')
        print(Amplitude,"Amplitude")
        # self.nepton_movment_limits = []
        success = self.moveit_class.go_start_pose()
        goal_pose = self.moveit_class.move_group.get_current_pose().pose
        goal_pose.position.z = goal_pose.position.z - 2*Amplitude
        goal_pose.position.x = goal_pose.position.x + 0.1
        # goal_pose.position.y = goal_pose.position.y + 0
        success = self.moveit_class.go_to_pose_goal(goal_pose)
        
        # self.nepton_movment_limits.append(self.position_manipulator.joint_angles)
        return EmptyResponse()

    def get_new_path_points(self,req):
        Amplitude = rospy.get_param('amplitude')
        print(Amplitude,"Amplitude")
        self.nepton_movment_limits = []
        success = self.moveit_class.go_start_pose()
        goal_pose = self.moveit_class.move_group.get_current_pose().pose
        print(goal_pose,"current_pose")
        goal_pose.position.z = goal_pose.position.z + Amplitude*math.sin(math.pi/2)
        # if Amplitude >0.24:
        #     goal_pose.position.x = goal_pose.position.x - 0
        # if Amplitude >0.25:
        #     goal_pose.position.x = goal_pose.position.x - 0.09
        print(goal_pose,"goal_pose")
        success = self.moveit_class.go_to_pose_goal(goal_pose)
        self.nepton_movment_limits.append(self.position_manipulator.joint_angles)
        

        # self.moveit_class.go_start_pose()
        # self.nepton_movment_limits.append(self.position_manipulator.joint_angles)
        if success:
            success = self.moveit_class.go_start_pose()
            goal_pose = self.moveit_class.move_group.get_current_pose().pose
            goal_pose.position.z = goal_pose.position.z + Amplitude*math.sin(-math.pi/2)
            # if Amplitude >0.24:
            #     goal_pose.position.x = goal_pose.position.x 
            # goal_pose.position.y = goal_pose.position.y + 0
            success = self.moveit_class.go_to_pose_goal(goal_pose)
            # print(goal_pose.position.z)
            # print(self.position_manipulator.joint_angles)
            self.nepton_movment_limits.append(self.position_manipulator.joint_angles)
            # self.moveit_class.go_start_pose()
            # self.nepton_movment_limits.append(self.position_manipulator.joint_angles)
            print(self.nepton_movment_limits,"self.nepton_movment_limits")
        else:
            rospy.loginfo("cant retch Amplitude")
            
        return EmptyResponse()

    def generat_path_to_robot(self,req):
        path = []
        movment_time_s = rospy.get_param('movment_time')
        cycle_time = rospy.get_param('cycle_time')
        print(type(movment_time_s))
        number_of_cycles = int(movment_time_s/cycle_time)
        print(number_of_cycles,"number_of_cycles")
        for cycle in range(number_of_cycles):
            for i in self.nepton_movment_limits:
                path.append(i)
        print(len(path),"len(path)")
        self.position_manipulator.follow_costum_path(path)
        return EmptyResponse()

    def go_to_start_pose(self,req):
        self.moveit_class.go_start_pose()
        return EmptyResponse()

    def print_corent_pose(self,req):
        print(self.moveit_class.move_group.get_current_pose().pose)
        return EmptyResponse()

    def get_data_from_ashkelon_file_data(self,req):
        file = open("/home/inbarm/catkin_ws/src/Nepton_Project/nepton_arm_project/src/recorded_data/ADCP_AST_data.txt", "r")
        content=file.readlines()
        file.close()
        self.file_dic = {}
        counter = 0
        for line_number in range(1,len(content)):
            line = content[line_number].split()
            ms = 0.5
            delta_z = 0
            if line_number > 1:
                previos_line = content[line_number-1].split()
                ms = float(line[5]) -float(previos_line[5])
                delta_z = float(line[7]) -float(previos_line[7])
            line_data = {"h":float(line[3]),"m":float(line[4]),"s":float(line[5]),"A":float(line[7]),"ms":ms, "duration":0.5 ,"delta_z":delta_z}
        
            if float(line[3])==5 and float(line[4]) >= 30: 
                # print(line_data,"line_data")
                self.file_dic[counter] = line_data
                counter = counter+1
        # print(len(self.file_dic),"len len(self.file_dic)")
        time_from_start = 0
        for line in range(0,len(self.file_dic)):
            duration = 0.5
            if line >1:
                if self.file_dic[line-1]["ms"] == self.file_dic[line]["ms"]:
                    duration =1
                if self.file_dic[line]["ms"] == 0:
                    duration = 0.5
                if self.file_dic[line]["ms"] > 1: 
                    duration = self.file_dic[line]["ms"]
                # print(self.file_dic[line])
                self.file_dic[line]["duration"] = duration
            time_from_start = time_from_start+duration
        # print(time_from_start,"time_from_start")
        return EmptyResponse()

    def recorde_joint_state_for_ashkelon_file_data(self,req):
        joint_val = []
        print(len(self.file_dic),"len(self.file_dic)")
        success = self.moveit_class.go_start_pose()
        start_pose = self.moveit_class.move_group.get_current_pose().pose
        goal_pose = copy.deepcopy(start_pose)
        for line in range(0,len(self.file_dic)) :
            print(line,"line")
            print(start_pose,"current_pose")
            print(self.file_dic[line])
            print(self.file_dic[line]["A"],self.file_dic[line]["A"]*Const)
            goal_pose.position.z = start_pose.position.z + self.file_dic[line]["A"]*Const
            # if Amplitude >0.24:
            #     goal_pose.position.x = goal_pose.position.x - 0
            # if Amplitude >0.25:
            #     goal_pose.position.x = goal_pose.position.x - 0.09
            print(goal_pose,"goal_pose")
            success = self.moveit_class.go_to_pose_goal(goal_pose)
            if  not success:
                break
            if success:
                joint_val.append(self.position_manipulator.joint_angles)

        with open('ashkelon_joint_val_1.pickle', 'wb') as handle:
            pickle.dump(joint_val, handle, protocol=pickle.HIGHEST_PROTOCOL)

        return EmptyResponse()

    def run_ashkelon_data(self,req):
        self.get_data_from_ashkelon_file_data(Empty)
        # rospy.ServiceProxy('/get_data_from_ashkelon_file_data', Empty)
        with open('/home/inbarm/catkin_ws/src/Nepton_Project/nepton_arm_project/src/ashkelon_joint_val.pickle', 'rb') as handle:
            path = pickle.load(handle)
        print(len(path),print(len(self.file_dic)))
        # print(path)
        self.position_manipulator.follow_ashkelon_path(path,self.file_dic)
        return EmptyResponse()

    def stop_gp8_movment(self,req):
        self.position_manipulator.follow_costum_path([])
        return EmptyResponse()

def main():
    a = nepton_program()
    rospy.Service("get_new_path_points",Empty, a.get_new_path_points)
    rospy.Service("generat_path_to_robot",Empty, a.generat_path_to_robot)
    rospy.Service("go_to_start_pose",Empty, a.go_to_start_pose)
    rospy.Service("test_max_limit",Empty,a.test_max_limit)
    rospy.Service("print_corent_pose",Empty,a.print_corent_pose)
    rospy.Service("get_data_from_ashkelon_file_data",Empty,a.get_data_from_ashkelon_file_data)
    rospy.Service("recorde_joint_state_for_ashkelon_file_data",Empty,a.recorde_joint_state_for_ashkelon_file_data)
    rospy.Service("run_ashkelon_data",Empty,a.run_ashkelon_data)
    rospy.Service("stop_gp8_movment",Empty,a.stop_gp8_movment)
    print("service is ready")
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True) 
    main()
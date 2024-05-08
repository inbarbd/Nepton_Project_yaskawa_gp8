# Nepton_Project

This program is base on [gp8_gazebo_moveit_pkg](gp8_gazebo_moveit_pkg) and [motoman_ps](https://github.com/MaxorPaxor/motoman_ps])</br>
Run on ubuntu 18.04 ros melodic</br>

Run gp8 controll sim or real:</br>
roslaunch nepton_arm_project gp8_robot_and_sim_moveit.launch </br>

For sim set arg gazebo_sim = True for real robot control gazebo_sim = False</br>

Run program gui control:</br>
cd ~/catkin_ws/src/Nepton_Project/nepton_arm_project/src </br>
python3 nepton_program_gui.py</br>

--

Servises:</br>

After setting the amplitud cycle time and test duration :</br>
rosservice call /get_new_path_points - set test joint path</br>
rosservice call /generat_path_to_robot - generate motoman path msg and sent to robot</br>
rosservice call /test_max_limit - for testing max robot limits if needed</br>
rosservice call /get_data_from_ashkelon_file_data - get recorded amplitud from the txt file</br>
rosservice call /recorde_joint_state_for_ashkelon_file_data - genetate robot joint path after test data.</br>
rosservice call /run_ashkelon_data - genetat motoman msg and send it to the to the robot</br>
rosservice call /stop_gp8_movment - stop real robot</br>


left gif - Gazebo Simulation </br>
center git - GUI Program Example </br>
right gif - Run Ashkelon Test </br>

<table>
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/gazebo_sim_ashkelon_data.gif" />
      <br/>
    </td>
    <td align="center">
    <!-- <caption>GUI Program Example</caption> -->
      <img align=center width=250 src="video/gui_program_example_1.gif" /> 
      <br/>
    </td>
    <td align="center">
    <!-- <caption>Run Ashkelon Test</caption> -->
      <img align=center width=250 src="video/run_ashkelon_test_1.gif" /> 
      <br/>
    </td>
  </tr>
</table>

--
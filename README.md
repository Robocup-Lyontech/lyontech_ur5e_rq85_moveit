## Settings once and for all

### On ROS Computer
- Follow install instructions : https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
- Install https://github.com/ros-industrial/robotiq


### On Communication Interface (UR Tablet)
install `external_control` URCap

install `rs485` URCap (and remove \*Gripper\* Cap if present or it will overide rs485 user com)

## After each robot power on

On Communication Interface (UR Tablet), go on Installation -> General -> Tool I/O 
and select Controlled by "User" with the following settings :
- Baud Rate : 115200
- Parity : None 
- Stop Bits : One
- RX Idle Chars : 1.5
- TX Idle Chars : 3.5
- Tool Output Voltage : 24V  

roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.210 use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR

- Communication Interface (UR Tablet), click below on "Play" and execute the external_control program

rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR


rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py 



top
position: [0.9126341978656214, -2.056477209130758, -4.931894604359762, -0.40775545061145024, 1.5036263465881348, 0.6962385177612305]




prepose
position: [1.233441177998678, -1.1601807636073609, -4.737365786229269, -0.04734094560656743, 0.6254684925079346, 0.45262598991394043]



pose
position: [1.233441177998678, -1.1601807636073609, -4.737365786229269, -0.04734094560656743, 0.6254684925079346, 0.45262598991394043]


postpose
position: [1.3934114615069788, -1.2207154792598267, -4.835902039204733, -0.46023281038317876, 0.3820152282714844, 0.6931338310241699]






# Warning

Premier terminal : Joint state with name: "robotiq_85_left_knuckle_joint" was received but not found in URDF

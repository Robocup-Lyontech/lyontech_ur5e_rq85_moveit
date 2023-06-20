## Settings once and for all

### On ROS Computer
- Follow install instructions : https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
- Install https://github.com/ros-industrial/robotiq


### On Communication Interface (UR Tablet)
install `external_control` URCap

install `rs485` URCap (and remove \*Gripper\* Cap if present or it will overide rs485 user com)


On Communication Interface (UR Tablet), go on Installation -> General -> Tool I/O 
and select Controlled by "User" with the following settings :
- Baud Rate : 115200
- Parity : None 
- Stop Bits : One
- RX Idle Chars : 1.5
- TX Idle Chars : 3.5
- Tool Output Voltage : 24V  

- In "Installation", click on "External Control" and set the IP address of the computer connected to the arm controller

## After each robot power on

### Connection

- Communication avec le controlleur du bras
    ```bash
    # Changer IP si besoin
    roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.210 use_tool_communication:=true tool_voltage:=24 tool_parity:=0 tool_baud_rate:=115200 tool_stop_bits:=1 tool_rx_idle_chars:=1.5 tool_tx_idle_chars:=3.5 tool_device_name:=/tmp/ttyUR
    ```

- Communication Interface (UR Tablet), click below on "Play" and execute the external_control program

### Gripper

- Run Gripper controller
    ```bash
    rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /tmp/ttyUR
    ```

- Run Gripper action controller
    ```bash
    roslaunch robotiq_2f_gripper_action_server robotiq_2f_gripper_action_server.launch
    ```

- Run Gripper action client 
    ```bash
    # OPTIONAL (if you move gripper with a terminal)
    rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py 
    # To Open/Close the gripper, first do Reset and Activate
    ```    

### MoveIt!

- Run Gripper controller
    ```bash
    roslaunch lyontech_ur5e_rq85_moveit move_group.launch sim:=false
    ```

- Joint State Publisher
    ```bash
    rosrun lyontech_ur5e_rq85_moveit gripper_joint_state_publisher.py
    ```

# Warning

Premier terminal : Joint state with name: "robotiq_85_left_knuckle_joint" was received but not found in URDF

# To sort ....


top
position: [0.9126341978656214, -2.056477209130758, -4.931894604359762, -0.40775545061145024, 1.5036263465881348, 0.6962385177612305]




prepose
position: [1.233441177998678, -1.1601807636073609, -4.737365786229269, -0.04734094560656743, 0.6254684925079346, 0.45262598991394043]



pose
position: [1.233441177998678, -1.1601807636073609, -4.737365786229269, -0.04734094560656743, 0.6254684925079346, 0.45262598991394043]


postpose
position: [1.3934114615069788, -1.2207154792598267, -4.835902039204733, -0.46023281038317876, 0.3820152282714844, 0.6931338310241699]








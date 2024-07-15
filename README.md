# sas_robot_driver_kuka

## Running the joint space example in CoppeliaSim

1. Download and unpack CoppeliaSim 4.1.0 Edu. https://coppeliarobotics.com/previousVersions

    i. *Common mistake: Downloading the wrong version*
    
    ii. *Common mistake: Downloading the `player` instead of `edu` version.*

2. Open the scene at `scenes/KUKAR820_410.ttt`

3. Start the simulation

    i. *Common mistake: Forgetting to start the simulation. This step is not optional.*

4. Run `ros2 launch sas_robot_driver_kuka sas_robot_driver_kuka_coppeliasim_joint_space_example.py`

If successful, it will output

```console
['LBR_iiwa_14_R820_joint1', 'LBR_iiwa_14_R820_joint2', 'LBR_iiwa_14_R820_joint3', 'LBR_iiwa_14_R820_joint4', 'LBR_iiwa_14_R820_joint5', 'LBR_iiwa_14_R820_joint6', 'LBR_iiwa_14_R820_joint7']
[INFO] [sas_robot_driver_ros_composer_node-1]: process started with pid [16823]
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.486678284] [kuka_coppeliasim_only]: ::Loading parameters from parameter server.
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.486741605] [kuka_coppeliasim_only]: ::Concatenating the joint limits from each robot.
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.486749960] [kuka_coppeliasim_only]: ::Parameters OK.
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.486753015] [kuka_coppeliasim_only]: ::Instantiating RobotDriverROSComposer.
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.486765435] [kuka_coppeliasim_only]: ::Instantiating RobotDriverROS.
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.486865427] [kuka_coppeliasim_only]: ::Initializing RobotDriverProvider with prefix kuka_coppeliasim_only
[sas_robot_driver_ros_composer_node-1] **************************************************************************
[sas_robot_driver_ros_composer_node-1] sas::Clock (c) Murilo M. Marinho (murilomarinho.info) 2016-2023 LGPLv3
[sas_robot_driver_ros_composer_node-1] **************************************************************************
[sas_robot_driver_ros_composer_node-1] **************************************************************************************
[sas_robot_driver_ros_composer_node-1] sas::RobotDriverServer (c) Murilo M. Marinho (murilomarinho.info) 2016-2023 LGPLv3
[sas_robot_driver_ros_composer_node-1] **************************************************************************************
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.489034854] [kuka_coppeliasim_only]: ::Waiting to connect with robot...
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.490169084] [kuka_coppeliasim_only]: ::Connected to CoppeliaSim
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.490186981] [kuka_coppeliasim_only]: ::Connected to robot.
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045346.490190147] [kuka_coppeliasim_only]: ::Initializing robot...
[sas_robot_driver_ros_composer_node-1] [INFO] [1721045347.282970657] [kuka_coppeliasim_only]: ::Robot initialized.
```

And the topics can be obtained with

```console
ros2 topic list | grep kuka_coppeliasim_only
```

resulting in

```console
/kuka_coppeliasim_only/get/home_states
/kuka_coppeliasim_only/get/joint_positions_max
/kuka_coppeliasim_only/get/joint_positions_min
/kuka_coppeliasim_only/get/joint_states
/kuka_coppeliasim_only/set/clear_positions
/kuka_coppeliasim_only/set/homing_signal
/kuka_coppeliasim_only/set/target_joint_forces
/kuka_coppeliasim_only/set/target_joint_positions
/kuka_coppeliasim_only/set/target_joint_velocities
```

## Working with the realtime kernel available in Ubuntu pro

The scheduling with 1ms did not work well on the stock kernel. I did try to chage the niceness of the entire process to no avail.

With the realtime kernel, surprisingly it worked without changing the scheduler. Nonetheless, I changed it to `SCHED_FIFO` just to be sure. Without any other load, the 1 ms schedule became quite robust. 

The installation with Ubuntu pro is extremely easy.

https://canonical-ubuntu-pro-client.readthedocs-hosted.com/en/latest/howtoguides/enable_realtime_kernel/

```console
sudo pro attach
sudo apt update && sudo apt install ubuntu-advantage-tools
sudo pro enable realtime-kernel
```

Use the following command to double check thread scheduling. Please note that only the FRI communication thread is realtime scheduled.

`ps -eLfc | grep FF`

which outputs, among many others,

`user   17545   17542   17559   15 FF   60 16:22 pts/1    00:00:21 /home/user/ros2_ws/install/sas_robot_driver_kuka/lib/sas_robot_driver_kuka/test_sas_driver_kuka`



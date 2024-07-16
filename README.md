# sas_robot_driver_kuka

## Running the joint space example in CoppeliaSim

1. Download and unpack CoppeliaSim 4.1.0 Edu. https://coppeliarobotics.com/previousVersions

    i. *Common mistake: Downloading the wrong version*
    
    ii. *Common mistake: Downloading the `player` instead of `edu` version.*

2. Open the scene at `scenes/KUKAR820_410.ttt`

3. Start the simulation

    i. *Common mistake: Forgetting to start the simulation. This step is not optional.*

    ii. _Common issue: After activating the node once, a second run might be unable to connect. Stop the simulation and restart it to solve the issue._ 

4. Split the terminator into four screens. Run, in each one (the order doesn't seem to matter).

   | `ros2 launch sas_robot_driver_kuka sas_robot_driver_kuka_composed_with_coppeliasim_launch.py` | `ros2 launch sas_robot_driver_kuka sas_robot_driver_kuka_dummy_robot_example_launch.py` |
   |-----------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------|
   | `ros2 run sas_robot_driver_kuka sas_robot_driver_kuka_joint_interface_example.py`             |                                                                                         |


## Working with the realtime kernel in Ubuntu Pro

The scheduling with 1ms did not work well on the stock kernel. I did try to chage the niceness of the entire process to no avail.

With the realtime kernel, surprisingly it worked without changing the scheduler. Nonetheless, I changed it to `SCHED_FIFO` just to be sure. Without any other load, the 1 ms schedule became quite robust. 

The installation with Ubuntu pro is extremely easy.

https://canonical-ubuntu-pro-client.readthedocs-hosted.com/en/latest/howtoguides/enable_realtime_kernel/

```console
sudo pro attach
sudo apt update && sudo apt install ubuntu-advantage-tools
sudo pro enable realtime-kernel
```

Use the following command to double-check thread scheduling. Please note that only the FRI communication thread is realtime scheduled.

`ps -eLfc | grep FF`

which outputs, among many others,

`user   17545   17542   17559   15 FF   60 16:22 pts/1    00:00:21 /home/user/ros2_ws/install/sas_robot_driver_kuka/lib/sas_robot_driver_kuka/test_sas_driver_kuka`



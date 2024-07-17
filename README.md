# sas_robot_driver_kuka

## Running the joint space example in CoppeliaSim

_Other versions of CoppeliaSim might be available. See `scenes/`._

1. Download and unpack CoppeliaSim 4.1.0 Edu. https://coppeliarobotics.com/previousVersions

    i. *Common mistake: Downloading the wrong version*
    
    ii. *Common mistake: Downloading the `player` instead of `edu` version.*

2. Open the scene at `scenes/KUKAR820_410.ttt`

3. Start the simulation

    i. *Common mistake: Forgetting to start the simulation. This step is not optional.*

    ii. _Common issue: After activating the node once, a second run might be unable to connect. Stop the simulation and restart it to solve the issue._ 

4. Run `ros2 launch sas_robot_driver_kuka coppeliasim_example_launch.py`

https://github.com/user-attachments/assets/715831f2-0504-4fb9-adc0-122521cd83c0

## Working with the real robot

For using the real robot, you **must** have the risk assessments in place. This guide is meant to be helpful but holds absolutely no liability whatsoever. More details are available in the software license.

This code will move the robot. Be sure that the workspace is free and safe for operation.

1. Be sure that the teaching pendant is in `AUT` mode. Please read the manual if you don't know what that means and the necessary conditions for this to be actionable.
2. Run the `PositionAndGMSReferencing` program once after each startup. 
3. Split the terminator into four screens. Now, the order matters.

| `a` | `b` |
|-----|-----|
| `c` | `d` |

4. In `a`, run the CoppeliaSim scene `scenes/KUKAR820_410.ttt` and start the simulation.
5. Get ready to do these two actions quickly. This is a safety measure. 
   1. Run the `MM_FRI_RobotApp` on the Kuka teaching pendant.
   2. In `b`, run `ros2 launch sas_robot_driver_kuka real_robot_example_launch.py`
   3. The robot should now be active. This means that the emergency button must be held at all times.
6. In `c`, run `ros2 launch sas_robot_driver_kuka composed_with_coppeliasim_launch.py`. This will connect the CoppeliaSim scene with the ros2 code.
7. In `d`, run `ros2 run sas_robot_driver_kuka joint_interface_example.py`. The robot will move in a sine wave in joint space, with respect to its initial joint values.

The statistics will be printed in the end. For example

```commandline
[INFO] [1721120552.202244265] [sas_robot_driver_kuka_joint_space_example_node_cpp]: ::Initializing RobotDriverClient with prefix sas_robot_driver_kuka_composed_with_coppeliasim
topic prefix = sas_robot_driver_kuka_composed_with_coppeliasim
joint positions = [0.29066365 0.29049773 0.29042559 0.29047892 0.29039368 0.29001315
 0.28983327]
Statistics for the entire loop
  Mean computation time: 0.000448198
  Mean idle time: 0.009589326
  Mean effective thread sampling time: 0.01003879
```

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

### Troubleshooting realtime problems

I have tested the driver with a considerable load in the machine, including CoppeliaSim (part of the example), Firefox, PyCharm, etc. open.

What I have found so far is that the connection is robust, with no interruptions. 

#### Unreliable network profiles

If the wired connection is in the `Connecting` status, it fails randomly, frequently, and fast. This is followed by a `Connection failure` Ubuntu error that pops up.
Although I cannot pinpoint the source of this issue, manually re-applying the settings **temporarily** fixed the issue.

To solve it permanently, in the system admin account, I deleted the wired network profile and redefined it with the correct configuration.


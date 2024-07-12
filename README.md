# sas_robot_driver_kuka

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



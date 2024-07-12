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


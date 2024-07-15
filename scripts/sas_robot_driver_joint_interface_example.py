"""
# Copyright (c) 2012-2024 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_kuka.
#
#    sas_robot_driver_kuka is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver_kuka is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver_kuka.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#   Based on `sas_robot_driver_joint_interface_example.py` from `sas_robot_driver`
#
# ################################################################
"""
import time

import numpy
from dqrobotics import *  # Despite what PyCharm might say, this is very much necessary or DQs will not be recognized
from dqrobotics.utils.DQ_Math import deg2rad

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_spin_some, rclcpp_shutdown
from sas_robot_driver import RobotDriverClient, RobotDriverServer


def main(args=None):
    try:
        rclcpp_init()
        node = rclcpp_Node("sas_robot_driver_kuka_joint_space_example_node_cpp")

        # Initialize the RobotDriverClient
        rdi = RobotDriverClient(node, 'real_kuka_1')

        # Wait for RobotDriverClient to be enabled
        while not rdi.is_enabled():
            rclcpp_spin_some(node)
            time.sleep(0.1)

        # Get topic information
        print(f"topic prefix = {rdi.get_topic_prefix()}")

        # Read the values sent by the RobotDriverServer
        joint_positions = rdi.get_joint_positions()
        print(f"joint positions = {joint_positions}")

        # Move the joints from their current state
        target_joint_positions = joint_positions + deg2rad([-0.5, -0.5, -0.5, -0.5, -0.5, -0.5, -0.5])
        rdi.send_target_joint_positions(target_joint_positions)

        # Wait until the joints move close enough
        while not numpy.allclose(target_joint_positions,rdi.get_joint_positions(),rtol=10e-1):
            rclcpp_spin_some(node)
            time.sleep(0.1)

        # Read the values sent by the RobotDriverServer
        joint_positions = rdi.get_joint_positions()
        print(f"joint positions = {joint_positions}")

        rclcpp_shutdown()

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print("Unhandled excepts", e)


if __name__ == '__main__':
    main()

/*
# Copyright (c) 2016-2024 Murilo Marques Marinho
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
#   Based on sas_robot_driver_example.cpp
#
# ################################################################*/
#include <rclcpp/rclcpp.hpp>
#include <sas_core/examples/sas_robot_driver_example.hpp>
#include <sas_core/eigen3_std_conversions.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <sas_common/sas_common.hpp>
#include "sas_robot_driver_kuka/sas_robot_driver_kuka.hpp"
#include <dqrobotics/utils/DQ_Math.h>

//#include<unistd.h>
//#include<sys/resource.h>
//#include<sys/syscall.h>
//#include<sys/types.h>


/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    //std::cout << setpriority(PRIO_PROCESS,0,-20) << std::endl;

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);

    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_kuka");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverKukaConfiguration configuration;
        node->get_parameter_or("robot_name",configuration.name,std::string("Kuka"));

        std::vector<double> joint_limits_min;
        std::vector<double> joint_limits_max;
        node->get_parameter_or("joint_limits_min",joint_limits_min,{-170,-120,-170,-120,-170,-120,-175});
        node->get_parameter_or("joint_limits_max",joint_limits_max,{170,120,170,120,170,120,175});
        configuration.joint_limits = {deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_min)),
                                      deg2rad(sas::std_vector_double_to_vectorxd(joint_limits_max))};

        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        node->get_parameter_or("thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec,0.001);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");


        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverKuka.");
        auto robot_driver_kuka = std::make_shared<sas::RobotDriverKuka>(configuration,
                                                                              &kill_this_process);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_kuka,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();

    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    sas::display_signal_handler_none_bug_info(node);
    return 0;
}

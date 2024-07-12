/*
# Copyright (c) 2016-2024 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver_kuka.
#
#    sas_denso_communcation is free software: you can redistribute it and/or modify
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
#   Based on sas_robot_driver_denso.cpp
#
# ################################################################*/

#include "sas_robot_driver_kuka/sas_robot_driver_kuka.hpp"
#include "joint_overlay_client.h"
#include <memory>


namespace sas
{

class RobotDriverKuka::Impl
{
public:
    std::shared_ptr<LBRJointCommandOverlayClient> trafo_client_;

    Impl()
    {
        trafo_client_ = std::make_shared<LBRJointCommandOverlayClient>();
    };

};

RobotDriverKuka::RobotDriverKuka(const RobotDriverKukaConfiguration& configuration, std::atomic_bool* break_loops):
RobotDriver(break_loops)
{
    impl_ = std::make_unique<RobotDriverKuka::Impl>();
    joint_limits_ = configuration.joint_limits;
}

RobotDriverKuka::~RobotDriverKuka()
{

}

VectorXd RobotDriverKuka::get_joint_positions()
{
    return impl_->trafo_client_->get_measured_joint_values();
}

void RobotDriverKuka::set_target_joint_positions(const VectorXd &desired_joint_positions_rad)
{
    impl_->trafo_client_->set_target_joint_values(desired_joint_positions_rad);
}

void RobotDriverKuka::connect()
{
    std::atomic_bool connection_state(false); //Unknown connection state
    fri_thread_ = std::thread(communication_thread_loop, impl_->trafo_client_, break_loops_, &connection_state);

    //We need the connection to be established before moving on.
    //However, we guarantee that this doesn't lock us with break_loops.
    while (!(*break_loops_) && !connection_state)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void RobotDriverKuka::disconnect()
{
    //To force the thread to shutdown if it hasn't already done so
    *break_loops_ = true;

    if (fri_thread_.joinable())
        fri_thread_.join();
}

void RobotDriverKuka::initialize()
{
    //Nothing to do
}

void RobotDriverKuka::deinitialize()
{
    //Nothing to do
}

}

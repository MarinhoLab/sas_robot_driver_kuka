#pragma once
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
#   Based on sas_robot_driver_denso.h
#
# ################################################################*/

#include <atomic>
#include <thread>

#include <sas_core/sas_robot_driver.hpp>

using namespace Eigen;

namespace sas
{
//Declared internally
class DriverBcap;

struct RobotDriverKukaConfiguration
{
    std::string name;
    std::tuple<VectorXd,VectorXd> joint_limits;
};

class RobotDriverKuka: public RobotDriver
{
private:
    RobotDriverKukaConfiguration configuration_;
    std::thread fri_thread_;

    //Implementation details that depend on FRI source files.
    class Impl;
    std::unique_ptr<Impl> impl_;
public:

    RobotDriverKuka(const RobotDriverKuka&)=delete;
    RobotDriverKuka()=delete;
    ~RobotDriverKuka();

    RobotDriverKuka(const RobotDriverKukaConfiguration& configuration, std::atomic_bool* break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    VectorXd get_joint_torques() override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

};
}

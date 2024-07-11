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
#include <vector>
#include <memory>

#include <dqrobotics/DQ.h>

#include <sas_core/sas_robot_driver.hpp>

using namespace DQ_robotics;
using namespace Eigen;

namespace sas
{
//Declared internally
class DriverBcap;

struct RobotDriverKukaConfiguration
{
    std::string ip_address;
    int port;
    double speed;
};


class RobotDriverKuka: public RobotDriver
{
private:
    RobotDriverKukaConfiguration configuration_;

    //BCAP driver
    //std::unique_ptr<DriverBcap> bcap_driver_;

    //Joint positions
    //VectorXd joint_positions_;
    //std::vector<double> joint_positions_buffer_;

    //DQ _homogenous_vector_to_dq(const VectorXd& homogenousvector) const;
    //VectorXd _dq_to_homogenous_vector(const DQ& pose) const;
    //VectorXd _get_end_effector_pose_homogenous_transformation();
    //double _sign(const double &a) const;

    void _connect(); //Throws std::runtime_error()

    void _motor_on(); //Throws std::runtime_error()
    void _motor_off() noexcept; //No exceptions should be thrown in the path to turn off the robot

    void _set_speed(const float& speed, const float& acceleration, const float& deacceleration); //Throws std::runtime_error()

    void _slave_mode_on(int mode); //Throws std::runtime_error()
    void _slave_mode_off() noexcept; //No exceptions should be thrown in the path to turn off the robot

public:
    //const static int SLAVE_MODE_JOINT_CONTROL;
    //const static int SLAVE_MODE_END_EFFECTOR_CONTROL;

    RobotDriverKuka(const RobotDriverKuka&)=delete;
    RobotDriverKuka()=delete;
    ~RobotDriverKuka();

    RobotDriverKuka(const RobotDriverKukaConfiguration& configuration, std::atomic_bool* break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& desired_joint_positions_rad) override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

};
}
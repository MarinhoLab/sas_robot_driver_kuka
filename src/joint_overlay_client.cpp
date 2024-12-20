//Heavily modified from the LBRJointSineOverlayApp by Kuka.
//Author: Murilo M. Marinho
//Email: murilo.marinho@manchester.ac.uk
/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software �KUKA Sunrise.Connectivity FRI Client SDK� is targeted to work in
conjunction with the �KUKA Sunrise.Connectivity FastRobotInterface� toolkit.
In the following, the term �software� refers to all material directly
belonging to the provided SDK �Software development kit�, particularly source
code, libraries, binaries, manuals and technical documentation.

COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2018
KUKA Deutschland GmbH
Augsburg, Germany

LICENSE

Redistribution and use of the software in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:
a) The software is used in conjunction with KUKA products only.
b) Redistributions of source code must retain the above copyright notice, this
list of conditions and the disclaimer.
c) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the disclaimer in the documentation and/or other
materials provided with the distribution. Altered source code of the
redistribution must be made available upon request with the distribution.
d) Modification and contributions to the original software provided by KUKA
must be clearly marked and the authorship must be stated.
e) Neither the name of KUKA nor the trademarks owned by KUKA may be used to
endorse or promote products derived from this software without specific prior
written permission.

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," without warranty of
any kind, including without limitation the warranties of merchantability,
fitness for a particular purpose and non-infringement.
KUKA makes no warranty that the Software is free of defects or is suitable for
any particular purpose. In no event shall KUKA be responsible for loss or
damages arising from the installation or use of the Software, including but
not limited to any indirect, punitive, special, incidental or consequential
damages of any character including, without limitation, damages for loss of
goodwill, work stoppage, computer failure or malfunction, or any and all other
commercial damages or losses.
The entire risk to the quality and performance of the Software is not borne by
KUKA. Should the Software prove defective, KUKA is not liable for the entire
cost of any service and repair.



\file
\version {1.16}
*/
#include <iostream>
#include <cstring>
#include "joint_overlay_client.h"
#include "friLBRState.h"
#include <sas_core/eigen3_std_conversions.hpp>

using namespace KUKA::FRI;
constexpr bool VERBOSE = false;

LBRJointCommandOverlayClient::~LBRJointCommandOverlayClient()
{
}


void LBRJointCommandOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
    LBRClient::onStateChange(oldState, newState);
    // (re)initialize sine parameters when entering Monitoring
    switch (newState)
    {
    case MONITORING_READY:
    {
        //This is available in the original example but its need here is unclear.
        _offset = 0.0;
        break;
    }
    default:
    {
        break;
    }
    }
}

VectorXd LBRJointCommandOverlayClient::get_measured_joint_values() const
{
    std::lock_guard<std::mutex> lock(mutex_measured_joint_values_);
    return sas::std_vector_double_to_vectorxd(measured_joint_values_);
}

VectorXd LBRJointCommandOverlayClient::get_measured_joint_torques() const
{
    std::lock_guard<std::mutex> lock(mutex_measured_joint_torques_);
    return sas::std_vector_double_to_vectorxd(measured_joint_torques_);
}

void LBRJointCommandOverlayClient::set_target_joint_values(const VectorXd& q)
{
    if (q.size() != 7)
        throw std::runtime_error("Wrong vector size in set_target_joint_values");

    std::lock_guard<std::mutex> lock(mutex_target_joint_values_);
    target_joint_values_ = sas::vectorxd_to_std_vector_double(q);
}

/**
 * @brief LBRJointCommandOverlayClient::command
 *
 * Ideally the only way to guarantee "command" does not lock for too long is to use "try-lock" locally
 * and "lock" in get_measured_joint_values() and set_target_joint_values(), even with the reduced scope.
 */
void LBRJointCommandOverlayClient::command()
{
    //std::scoped_lock<std::mutex, std::mutex> lock(mutex_measured_joint_values_, mutex_target_joint_values_);

    double joint_position_array[LBRState::NUMBER_OF_JOINTS];
    memcpy(joint_position_array, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

    { // Measured joint values mutex scope
        std::lock_guard lock(mutex_measured_joint_values_);
        measured_joint_values_ = std::vector<double>(joint_position_array, joint_position_array + LBRState::NUMBER_OF_JOINTS);
    }

    // Certain robot types, e.g. the LBR iiwa, have a joint torque sensor in each axis
    // which measures the torque acting on the axis. The interface ITorqueSensiti-
    // veRobot contains the methods required for polling sensor data from the robot.

    // The measured torque values can be polled and evaluated in the application via the method getMeasuredTorque
    double measured_torque_array[LBRState::NUMBER_OF_JOINTS];
    memcpy(measured_torque_array, robotState().getMeasuredTorque(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

    // The external torque information is only valid if it's properly configured in the system, which is not the case now.
    // KUKA_SunriseOS_111_SI_en.pdf, page 371
    { // Measured joint torques mutex scope
        std::lock_guard lock(mutex_measured_joint_torques_);
        measured_joint_torques_ = std::vector<double>(measured_torque_array, measured_torque_array + LBRState::NUMBER_OF_JOINTS);
    }


    { // Target joint values mutex scope
        std::lock_guard lock(mutex_target_joint_values_);
        // Initialize target joint values if they are empty
        if (target_joint_values_.size() == 0)
            target_joint_values_ = measured_joint_values_; // This only reads the state of "measured" and can't be run while the lock above is active.
        robotCommand().setJointPosition(&target_joint_values_[0]);
    }

    if (VERBOSE)
    {
        for (const auto& joint_value : measured_joint_values_)
        {
            std::cout << joint_value;
        }
        std::cout << std::endl;
    }
}



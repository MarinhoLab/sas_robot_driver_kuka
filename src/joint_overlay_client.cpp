//Modified from the LBRJointSineOverlayApp by Kuka.
//Author: Murilo M. Marinho
//Email: murilo.marinho@manchester.ac.uk
/**

The following license terms and conditions apply, unless a redistribution
agreement or other license is obtained by KUKA Deutschland GmbH, Augsburg, Germany.

SCOPE

The software “KUKA Sunrise.Connectivity FRI Client SDK” is targeted to work in
conjunction with the “KUKA Sunrise.Connectivity FastRobotInterface” toolkit.
In the following, the term “software” refers to all material directly
belonging to the provided SDK “Software development kit”, particularly source
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

#include <QtCore/QByteArray>
#include <QtCore/QDataStream>
#include <QtCore/QIODevice>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QHostAddress>

using namespace KUKA::FRI;

constexpr bool VERBOSE = false;
//******************************************************************************


//******************************************************************************
LBRJointCommandOverlayClient::~LBRJointCommandOverlayClient()
{
}

//******************************************************************************
void LBRJointCommandOverlayClient::onStateChange(ESessionState oldState, ESessionState newState)
{
	LBRClient::onStateChange(oldState, newState);
	// (re)initialize sine parameters when entering Monitoring
	switch (newState)
	{
	case MONITORING_READY:
	{
		_offset = 0.0;

		break;
	}
	default:
	{
		break;
	}
	}
}

std::vector<double> LBRJointCommandOverlayClient::get_measured_joint_values() const
{
	return measured_joint_values_;
}

double LBRJointCommandOverlayClient::get_measured_joint_value(int index) const
{
	return measured_joint_values_.at(index);
}

void LBRJointCommandOverlayClient::set_joint_value_update_callback(const UpdateCallbackFunctionType& callback)
{
	joint_value_update_callback_ = callback;
}

void LBRJointCommandOverlayClient::set_target_joint_value_update_callback(const UpdateCallbackFunctionType& callback)
{
	target_joint_value_update_callback_ = callback;
}

void LBRJointCommandOverlayClient::set_target_joint_value(const double& q, int index)
{
	target_joint_values_.at(index) = q;
}

void LBRJointCommandOverlayClient::set_target_joint_values(const std::vector<double>& q)
{
	if (q.size() != 7)
		throw std::runtime_error("Wrong vector size in set_target_joint_values");
	target_joint_values_ = q;
}

//******************************************************************************
void LBRJointCommandOverlayClient::command()
{
	double jointPos[LBRState::NUMBER_OF_JOINTS];

	/// The original exmaple was with IpoJointPositions, but that did not work on mine.
	// memcpy(jointPos, robotState().getIpoJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));

	memcpy(jointPos, robotState().getMeasuredJointPosition(), LBRState::NUMBER_OF_JOINTS * sizeof(double));
	measured_joint_values_ = std::vector<double>(jointPos, jointPos + 7);

	if (VERBOSE)
	{
		for (const auto& joint_value : measured_joint_values_)
		{
			std::cout << joint_value;
		}
		std::cout << std::endl;
	}

	// Initialize target joint values if they are empty
	if (target_joint_values_.size() == 0)
		target_joint_values_ = measured_joint_values_;

	if (joint_value_update_callback_ != nullptr)
		joint_value_update_callback_(measured_joint_values_);
	if (target_joint_value_update_callback_ != nullptr)
		target_joint_value_update_callback_(target_joint_values_);

	robotCommand().setJointPosition(&target_joint_values_[0]);

    //TODO Update ROS state with the current joint positions
    //measured_joint_values_
}
//******************************************************************************
// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif

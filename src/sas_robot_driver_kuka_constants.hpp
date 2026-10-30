/*
# Copyright (c) 2012-2026 Murilo Marques Marinho
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
*/

#pragma once
#include <vector>
#include <eigen3/Eigen/Core>
#include <sas_core/eigen3_std_conversions.hpp>


namespace sas
{
namespace kuka_r820_constants
{

const std::vector<double> joint_limits_min_deg_std{-170.,-120.,-170.,-120.,-170.,-120.,-175.};
const std::vector<double> joint_limits_max_deg_std{170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0};
const Eigen::VectorXd joint_limits_min_deg_eigen = sas::std_vector_double_to_vectorxd(joint_limits_min_deg_std);
const Eigen::VectorXd joint_limits_max_deg_eigen = sas::std_vector_double_to_vectorxd(joint_limits_max_deg_std);

}
}

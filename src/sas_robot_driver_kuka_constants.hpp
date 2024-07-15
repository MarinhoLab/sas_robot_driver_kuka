#pragma once
#include <vector>
#include <eigen3/Eigen/Core>
#include <sas_core/eigen3_std_conversions.hpp>

using namespace Eigen;

namespace sas
{
namespace kuka_r820_constants
{

const std::vector<double> joint_limits_min_deg_std{-170.,-120.,-170.,-120.,-170.,-120.,-175.};
const std::vector<double> joint_limits_max_deg_std{170.0, 120.0, 170.0, 120.0, 170.0, 120.0, 175.0};
const VectorXd joint_limits_min_deg_eigen = sas::std_vector_double_to_vectorxd(joint_limits_min_deg_std);
const VectorXd joint_limits_max_deg_eigen = sas::std_vector_double_to_vectorxd(joint_limits_max_deg_std);

}
}

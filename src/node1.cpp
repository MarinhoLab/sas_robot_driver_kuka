//Template from https://ros2-tutorial.readthedocs.io/en/latest/
#include "node1.hpp"

/**
 * @brief Node1::Node1 Default constructor.
 */
Node1::Node1():
    rclcpp::Node("node1"),
    timer_period_(0.5),
    print_count_(0)
{
    timer_ = create_wall_timer(
                std::chrono::milliseconds(long(timer_period_*1e3)),
                std::bind(&Node1::_timer_callback, this)
                );
}

/**
 * @brief Node1::_timer_callback periodically prints class info using RCLCPP_INFO.
 */
void Node1::_timer_callback()
{
    RCLCPP_INFO_STREAM(get_logger(),
                       std::string("Printed ") +
                       std::to_string(print_count_) +
                       std::string(" times.")
                       );
    print_count_++;
}

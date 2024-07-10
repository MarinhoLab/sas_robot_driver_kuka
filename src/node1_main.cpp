//Template from https://ros2-tutorial.readthedocs.io/en/latest/
#include <rclcpp/rclcpp.hpp>

#include "node1.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    try
    {
        auto node = std::make_shared<Node1>();

        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        std::cerr << std::string("::Exception::") << e.what();
    }

    return 0;
}

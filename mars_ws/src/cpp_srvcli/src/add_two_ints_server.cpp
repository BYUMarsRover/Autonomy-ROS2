#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("HelloMyDudes"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("HelloMyDudes"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); // Initializes ROS 2 C++ client library

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server"); // Creates a node named add_two_ints_server

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add); // Creates a service named add_two_ints for that node and automatically advertises it over the networks with the &add method:

  RCLCPP_INFO(rclcpp::get_logger("HelloMyDudes"), "Ready to add two ints."); // Prints a log message when it’s ready:

  rclcpp::spin(node); // Spins the node, making the service available.
  rclcpp::shutdown();
  
}
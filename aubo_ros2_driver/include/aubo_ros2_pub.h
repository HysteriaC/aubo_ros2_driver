#ifndef DYNAMIC_PUBLISHER_SERVICE_ACTION_HPP
#define DYNAMIC_PUBLISHER_SERVICE_ACTION_HPP

#include "rclcpp/rclcpp.hpp"
#include <map>
#include <string>
#include <memory>
#include <vector>
#include <filesystem>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace fs = std::filesystem;

class DynamicPublisherServiceAction : public rclcpp::Node
{
public:
    DynamicPublisherServiceAction();

private:
    void load_message_publishers(const std::string& msg_directory);
    void load_service_servers(const std::string& srv_directory);
    void load_action_servers(const std::string& action_directory);

    void publish_messages();
    void handle_service_request(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal);
    void handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topic_publishers_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_robot_mode_service_;
    rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr move_robot_action_server_;

    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // DYNAMIC_PUBLISHER_SERVICE_ACTION_HPP


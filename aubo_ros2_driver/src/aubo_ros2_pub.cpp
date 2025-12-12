#include "rclcpp/rclcpp.hpp"
#include <map>
#include <string>
#include <memory>
#include "aubo_interfaces.h"
#include <vector>
#include <iostream>
#include <filesystem> 

namespace fs = std::filesystem;

class DynamicPublisherServiceAction : public rclcpp::Node
{
public:
    DynamicPublisherServiceAction() : Node("dynamic_publisher_service_action")
    {
        load_message_publishers("aubo_interfaces/msg"); 
        load_service_servers("aubo_interfaces/srv");
        load_action_servers("aubo_interfaces/action"); 
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&DynamicPublisherServiceAction::publish_messages, this));
    }

private:
    void load_message_publishers(const std::string& msg_directory)
    {
        for (const auto& entry : fs::directory_iterator(msg_directory))
        {
            if (entry.path().extension() == ".msg")
            {
                std::string topic_name = entry.path().stem().string();
                topic_publishers_[topic_name] = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
                RCLCPP_INFO(this->get_logger(), "Created publisher for topic: %s", topic_name.c_str());
            }
        }
    }

    void load_service_servers(const std::string& srv_directory)
    {
        for (const auto& entry : fs::directory_iterator(srv_directory))
        {
            if (entry.path().extension() == ".srv")
            {
                std::string service_name = entry.path().stem().string();
                if (service_name == "set_robot_mode")
                {
                    set_robot_mode_service_ = this->create_service<std_srvs::srv::SetBool>(
                        service_name, std::bind(&DynamicPublisherServiceAction::handle_service_request, this, std::placeholders::_1, std::placeholders::_2));
                    RCLCPP_INFO(this->get_logger(), "Created service for: %s", service_name.c_str());
                }
            }
        }
    }

    void load_action_servers(const std::string& action_directory)
    {
        for (const auto& entry : fs::directory_iterator(action_directory))
        {
            if (entry.path().extension() == ".action")
            {
                std::string action_name = entry.path().stem().string();
                if (action_name == "move_robot")
                {
                    move_robot_action_server_ = rclcpp_action::create_server<example_interfaces::action::Fibonacci>(
                        this, action_name,
                        std::bind(&DynamicPublisherServiceAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                        std::bind(&DynamicPublisherServiceAction::handle_cancel, this, std::placeholders::_1),
                        std::bind(&DynamicPublisherServiceAction::handle_accepted, this, std::placeholders::_1));
                    RCLCPP_INFO(this->get_logger(), "Created action server for: %s", action_name.c_str());
                }
            }
        }
    }

    void publish_messages()
    {
        for (auto &pair : topic_publishers_)
        {
            auto message = std_msgs::msg::String();
            message.data = "Sample message for " + pair.first;
            pair.second->publish(message);
            RCLCPP_INFO(this->get_logger(), "Publishing to %s: %s", pair.first.c_str(), message.data.c_str());
        }
    }

    void handle_service_request(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service called with request: %d", request->data);
        response->success = true;
    }

    rclcpp_action::Server<example_interfaces::action::Fibonacci>::SharedPtr move_robot_action_server_;

    void handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const example_interfaces::action::Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal for move_robot_action");
    }

    void handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Action canceled");
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<example_interfaces::action::Fibonacci>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Action goal accepted");
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topic_publishers_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_robot_mode_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicPublisherServiceAction>());
    rclcpp::shutdown();
    return 0;
}


#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "scanner_action_interfaces/action/scan_sweep.hpp"

using namespace std::chrono_literals;
using ScanSweep = scanner_action_interfaces::action::ScanSweep;
using GoalHandleScan = rclcpp_action::ClientGoalHandle<ScanSweep>;

class ScannerActionClient : public rclcpp::Node
{
public:
    ScannerActionClient()
    : Node("scanner_action_client")
    {
        client_ = rclcpp_action::create_client<ScanSweep>(
            this,
            "scan_sweep");
    }

    void send_goal()
    {
        RCLCPP_INFO(this->get_logger(), "Menunggu action server...");
        client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Terhubung ke server. Mengirim goal...\n");

        ScanSweep::Goal goal_msg;
        goal_msg.trigger = true;
        goal_msg.start_position_deg = -75.0;
        goal_msg.stop_position_deg  = -125.0;

        auto options =
            rclcpp_action::Client<ScanSweep>::SendGoalOptions();

        options.feedback_callback =
            std::bind(&ScannerActionClient::feedback_callback,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2);

        options.result_callback =
            std::bind(&ScannerActionClient::result_callback,
                      this,
                      std::placeholders::_1);

        client_->async_send_goal(goal_msg, options);
    }

    bool is_done() const
    {
        return done_;
    }

private:
    rclcpp_action::Client<ScanSweep>::SharedPtr client_;
    bool done_ = false;

    void feedback_callback(
        GoalHandleScan::SharedPtr,
        const std::shared_ptr<const ScanSweep::Feedback> feedback)
    {
        std::string phase = feedback->current_phase;
        double pos = feedback->current_position_deg;
        double pct = feedback->progress_percent;

        std::cout << "\r  ["
                  << std::left << std::setw(12) << phase
                  << "| "
                  << std::fixed << std::setw(5)
                  << std::setprecision(1) << pct << "%] "
                  << "pos="
                  << std::showpos << std::setw(7)
                  << std::setprecision(2) << pos
                  << "°  "
                  << std::flush;
    }

    void result_callback(
        const GoalHandleScan::WrappedResult & result)
    {
        std::cout << std::endl;

        if (result.code ==
            rclcpp_action::ResultCode::SUCCEEDED &&
            result.result->success)
        {
            RCLCPP_INFO(this->get_logger(),
                "\nSUKSES: %s",
                result.result->message.c_str());
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                "\nGAGAL: %s",
                result.result->message.c_str());
        }

        done_ = true;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<ScannerActionClient>();

    std::cout << "Sending GOAL to Sweep Action Server" << std::endl;

    client->send_goal();

    // 1:1 behavior dengan spin_once loop Python
    while (rclcpp::ok() && !client->is_done())
    {
        rclcpp::spin_some(client);
        std::this_thread::sleep_for(100ms);
    }

    rclcpp::shutdown();
    return 0;
}
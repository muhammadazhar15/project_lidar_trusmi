#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>
#include <iomanip>
#include <iostream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "scanner_action_interfaces/action/scan_sweep.hpp"
#include "scanner_assembler/srv/assembler_query.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>

#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Geometry>

using namespace std::chrono_literals;
using ScanSweep = scanner_action_interfaces::action::ScanSweep;
using GoalHandleScan = rclcpp_action::ServerGoalHandle<ScanSweep>;

static constexpr double INTERVAL_S = 0.1; //0.1s
static constexpr double DEG_FAST = 3.0;
static constexpr double DEG_SLOW = 0.3;
static constexpr double POS_START_DEG = 0.0;
static constexpr double REACH_TOLERANCE = 0.5;

double deg2rad(double d){ return d * M_PI / 180.0; }
double rad2deg(double r){ return r * 180.0 / M_PI; }

class ScannerActionServer : public rclcpp::Node
{
public:
    ScannerActionServer()
    : Node("scanner_action_server")
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);

        assembled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/assembled_cloud", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ScannerActionServer::imuCallback, this, std::placeholders::_1));

        assembler_client_ =
            this->create_client<scanner_assembler::srv::AssemblerQuery>("/assembler_service");

        action_server_ = rclcpp_action::create_server<ScanSweep>(
            this,
            "scan_sweep",
            std::bind(&ScannerActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ScannerActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ScannerActionServer::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options()
        );

        // CropBox parameters (dynamic)
        rcl_interfaces::msg::ParameterDescriptor desc_minx;
        desc_minx.description = "Crop min X";
        desc_minx.read_only = false;          // bisa diubah live
        rcl_interfaces::msg::FloatingPointRange range_minx;
        range_minx.from_value = -6.0;
        range_minx.to_value   = 0.0;
        range_minx.step       = 0.01;
        desc_minx.floating_point_range.push_back(range_minx);
        this->declare_parameter("crop_min_x", -3.0, desc_minx);

        rcl_interfaces::msg::ParameterDescriptor desc_maxx;
        desc_maxx.description = "Crop max X";
        desc_maxx.read_only = false;
        rcl_interfaces::msg::FloatingPointRange range_maxx;
        range_maxx.from_value = 0.0;
        range_maxx.to_value   = 6.0;
        range_maxx.step       = 0.01;
        desc_maxx.floating_point_range.push_back(range_maxx);
        this->declare_parameter("crop_max_x", 3.0, desc_maxx);

        rcl_interfaces::msg::ParameterDescriptor desc_miny;
        desc_miny.description = "Crop min Y";
        desc_miny.read_only = false;
        rcl_interfaces::msg::FloatingPointRange range_miny;
        range_miny.from_value = -6.0;
        range_miny.to_value   = 0.0;
        range_miny.step       = 0.01;
        desc_miny.floating_point_range.push_back(range_miny);
        this->declare_parameter("crop_min_y", -3.0, desc_miny);

        rcl_interfaces::msg::ParameterDescriptor desc_maxy;
        desc_maxy.description = "Crop max Y";
        desc_maxy.read_only = false;
        rcl_interfaces::msg::FloatingPointRange range_maxy;
        range_maxy.from_value = 0.0;
        range_maxy.to_value   = 6.0;
        range_maxy.step       = 0.01;
        desc_maxy.floating_point_range.push_back(range_maxy);
        this->declare_parameter("crop_max_y", 3.0, desc_maxy);

        rcl_interfaces::msg::ParameterDescriptor desc_minz;
        desc_minz.description = "Crop min Z";
        desc_minz.read_only = false;
        rcl_interfaces::msg::FloatingPointRange range_minz;
        range_minz.from_value = -6.0;
        range_minz.to_value   = 0.0;
        range_minz.step       = 0.01;
        desc_minz.floating_point_range.push_back(range_minz);
        this->declare_parameter("crop_min_z", 0.0, desc_minz);

        rcl_interfaces::msg::ParameterDescriptor desc_maxz;
        desc_maxz.description = "Crop max Z";
        desc_maxz.read_only = false;
        rcl_interfaces::msg::FloatingPointRange range_maxz;
        range_maxz.from_value = 0.0;
        range_maxz.to_value   = 6.0;
        range_maxz.step       = 0.01;
        desc_maxz.floating_point_range.push_back(range_maxz);
        this->declare_parameter("crop_max_z", 4.0, desc_maxz);

        rcl_interfaces::msg::ParameterDescriptor desc_yaw;
        desc_yaw.description = "Crop yaw rotation in degrees";
        desc_yaw.read_only = false;
        rcl_interfaces::msg::FloatingPointRange range_yaw;
        range_yaw.from_value = -180.0;
        range_yaw.to_value   = 180.0;
        range_yaw.step       = 1.0;
        desc_yaw.floating_point_range.push_back(range_yaw);
        this->declare_parameter("crop_yaw_deg", 0.0, desc_yaw);

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ScannerActionServer::paramCallback, this, std::placeholders::_1)
        );
        cropbox_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cropbox_marker", 10);

        this->get_parameter("crop_min_x", crop_min_x_);
        this->get_parameter("crop_max_x", crop_max_x_);
        this->get_parameter("crop_min_y", crop_min_y_);
        this->get_parameter("crop_max_y", crop_max_y_);
        this->get_parameter("crop_min_z", crop_min_z_);
        this->get_parameter("crop_max_z", crop_max_z_);
        this->get_parameter("crop_yaw_deg", crop_yaw_deg_);
        publishCropBoxMarker();

        RCLCPP_INFO(this->get_logger(), "ScannerActionServer siap. Menunggu goal...");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr assembled_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Client<scanner_assembler::srv::AssemblerQuery>::SharedPtr assembler_client_;
    rclcpp_action::Server<ScanSweep>::SharedPtr action_server_;

    double crop_min_x_, crop_max_x_;
    double crop_min_y_, crop_max_y_;
    double crop_min_z_, crop_max_z_;
    double crop_yaw_deg_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cropbox_marker_pub_;

    double current_pos_rad_ = 0.0;
    double pos_start_deg_ = -75;
    double pos_stop_deg_  = -125;

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pos_rad_ = roll;
    }

    void print_header()
    {
        std::cout << std::setw(5) << "Step" << " │ "
                  << std::setw(12) << "Fase" << " │ "
                  << std::setw(10) << "Setpoint" << " │ "
                  << std::setw(10) << "Aktual GZ" << " │ "
                  << std::setw(8) << "Error" << " │ "
                  << std::setw(8) << "Progress\n";
    }

    void print_row(int step, const std::string &phase,
                   double setpoint_deg, double actual_deg,
                   double progress)
    {
        double error = actual_deg - setpoint_deg;

        std::cout << std::setw(5) << step << " │ "
                  << std::setw(12) << phase << " │ "
                  << std::fixed << std::setprecision(2)
                  << std::setw(9) << setpoint_deg << "° │ "
                  << std::setw(9) << actual_deg << "° │ "
                  << std::setw(7) << error << "° │ "
                  << std::setw(5) << std::setprecision(1)
                  << progress << "% " << "\n";
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ScanSweep::Goal> goal)
    {
        if (!goal->trigger)
            return rclcpp_action::GoalResponse::REJECT;

        pos_start_deg_ = goal->start_position_deg;
        pos_stop_deg_  = goal->stop_position_deg;

        RCLCPP_INFO(this->get_logger(), "Goal diterima — memulai sweep.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleScan>)
    {
        RCLCPP_INFO(this->get_logger(), "Cancel diminta.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleScan> goal_handle)
    {
        std::thread{std::bind(&ScannerActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleScan> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Eksekusi sweep dimulai.");

        auto feedback = std::make_shared<ScanSweep::Feedback>();
        auto result   = std::make_shared<ScanSweep::Result>();

        int steps_fast_go  = std::abs(pos_start_deg_ - POS_START_DEG) / DEG_FAST;
        int steps_sweeping = std::abs(pos_stop_deg_ - pos_start_deg_) / DEG_SLOW;
        int steps_fast_ret = std::abs(POS_START_DEG - pos_stop_deg_) / DEG_FAST;
        int total_steps    = steps_fast_go + steps_sweeping + steps_fast_ret;
        int step_count     = 0;

        rclcpp::Rate rate(1.0 / INTERVAL_S);

        std::vector<std::tuple<std::string,double,double>> segments = {
            {"fast_go", pos_start_deg_, DEG_FAST},
            {"sweeping", pos_stop_deg_, DEG_SLOW},
            {"fast_return", POS_START_DEG, DEG_FAST}
        };

        double setpoint_deg = rad2deg(current_pos_rad_);

        print_header();

        auto req_tmp = std::make_shared<scanner_assembler::srv::AssemblerQuery::Request>();
        req_tmp->clear_data = true;

        rclcpp::Time sweep_begin_time, sweep_end_time;

        for (auto &seg : segments)
        {
            std::string phase;
            double target, step_deg;
            std::tie(phase, target, step_deg) = seg;

            if (phase == "sweeping")
            {
                sweep_begin_time = this->now();
                req_tmp->begin = sweep_begin_time;
            }

            while (rclcpp::ok())
            {
                if (goal_handle->is_canceling())
                {
                    result->success = false;
                    result->message = "Sweep dibatalkan.";
                    goal_handle->canceled(result);
                    return;
                }

                double remaining = std::fabs(target - setpoint_deg);
                if (remaining <= REACH_TOLERANCE)
                {
                    setpoint_deg = target;
                    publishCommand(setpoint_deg);
                    break;
                }

                if (target > setpoint_deg){
                    setpoint_deg += step_deg;
                    setpoint_deg = std::min(setpoint_deg, target);
                }   
                else if(target < setpoint_deg){
                    setpoint_deg -= step_deg;
                    setpoint_deg = std::max(setpoint_deg, target);
                }
                    

                publishCommand(setpoint_deg);
                step_count++;

                double actual_deg = rad2deg(current_pos_rad_);
                double progress =
                    std::min(100.0,
                        (static_cast<double>(step_count) /
                        static_cast<double>(total_steps)) * 100.0);

                print_row(step_count, phase, setpoint_deg, actual_deg, progress);

                if (phase == "sweeping")
                {
                    req_tmp->end = this->now();
                    auto future = assembler_client_->async_send_request(req_tmp);

                    if (future.wait_for(5s) ==
                        std::future_status::ready)
                    {
                        auto response = future.get();
                        // assembled_pub_->publish(response->cloud);
                        assembled_pub_->publish(filterCloud(response->cloud));
                    }
                }

                feedback->current_position_deg = actual_deg;
                feedback->current_phase = phase;
                feedback->progress_percent = progress;
                goal_handle->publish_feedback(feedback);

                rate.sleep();
            }

            if (phase == "sweeping")
                sweep_end_time = this->now();
        }

        if (sweep_begin_time.nanoseconds() > 0 && sweep_end_time.nanoseconds() > 0)
        {
            auto req = std::make_shared<scanner_assembler::srv::AssemblerQuery::Request>();
            req->begin = sweep_begin_time;
            req->end   = sweep_end_time;
            req->clear_data = false;

            auto future = assembler_client_->async_send_request(req);
            if (future.wait_for(5s) ==
                std::future_status::ready)
            {
                auto response = future.get();
                assembled_pub_->publish(filterCloud(response->cloud));
            }
        }

        result->success = true;
        result->message = "Sweep selesai!";
        goal_handle->succeed(result);

        RCLCPP_INFO(this->get_logger(), "MISI SELESAI!!");
    }

    void publishCommand(double deg)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {deg2rad(deg)};
        cmd_pub_->publish(msg);
    }

    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> &params)
    {
        for (const auto &param : params)
        {
            if (param.get_name() == "crop_min_x") crop_min_x_ = param.as_double();
            if (param.get_name() == "crop_max_x") crop_max_x_ = param.as_double();
            if (param.get_name() == "crop_min_y") crop_min_y_ = param.as_double();
            if (param.get_name() == "crop_max_y") crop_max_y_ = param.as_double();
            if (param.get_name() == "crop_min_z") crop_min_z_ = param.as_double();
            if (param.get_name() == "crop_max_z") crop_max_z_ = param.as_double();
            if (param.get_name() == "crop_yaw_deg") crop_yaw_deg_ = param.as_double();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        publishCropBoxMarker();
        return result;
    }

    sensor_msgs::msg::PointCloud2 filterCloud(const sensor_msgs::msg::PointCloud2 &input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(input, *cloud);

        // =============================
        // 1️⃣ Voxel Grid (4cm)
        // =============================
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.04f, 0.04f, 0.04f);
        vg.filter(*voxel_filtered);

        // =============================
        // 2️⃣ Statistical Outlier Removal
        // =============================
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(voxel_filtered);
        sor.setMeanK(20);
        sor.setStddevMulThresh(1.0);
        sor.filter(*sor_filtered);

        // =============================
        // 3️⃣ CropBox (Dynamic + Yaw)
        // =============================
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::CropBox<pcl::PointXYZ> crop;
        crop.setInputCloud(sor_filtered);

        crop.setMin(Eigen::Vector4f(crop_min_x_, crop_min_y_, crop_min_z_, 1.0));
        crop.setMax(Eigen::Vector4f(crop_max_x_, crop_max_y_, crop_max_z_, 1.0));

        float yaw_rad = deg2rad(crop_yaw_deg_);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ()));

        crop.setTransform(transform);
        crop.filter(*cropped);

        // Convert back to ROS2
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cropped, output);
        output.header = input.header;

        return output;
    }

    void publishCropBoxMarker()
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "map";  // sesuaikan frame
        marker.header.stamp = this->now();
        marker.ns = "cropbox";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Hitung ukuran box
        double size_x = crop_max_x_ - crop_min_x_;
        double size_y = crop_max_y_ - crop_min_y_;
        double size_z = crop_max_z_ - crop_min_z_;

        marker.scale.x = size_x;
        marker.scale.y = size_y;
        marker.scale.z = size_z;

        // Hitung center box
        marker.pose.position.x = (crop_max_x_ + crop_min_x_) / 2.0;
        marker.pose.position.y = (crop_max_y_ + crop_min_y_) / 2.0;
        marker.pose.position.z = (crop_max_z_ + crop_min_z_) / 2.0;

        // Rotasi yaw
        double yaw_rad = deg2rad(crop_yaw_deg_);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);

        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // Warna transparan hijau
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;

        marker.lifetime = rclcpp::Duration::from_seconds(0.0);

        cropbox_marker_pub_->publish(marker);
    }
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScannerActionServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
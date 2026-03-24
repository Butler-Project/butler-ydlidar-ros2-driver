#include "tmini_driver_node/tmini_lidar.hpp"

#include <cmath>
#include <numbers>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class TminiDriverNode : public rclcpp::Node {
public:
    explicit TminiDriverNode(const rclcpp::NodeOptions& options)
        : Node("tmini_driver_node", options) {

        // Declare parameters
        declare_parameter("port", "/dev/lidar");
        declare_parameter("frame_id", "lidar");
        declare_parameter("baud_rate", 230400);
        declare_parameter("angle_min", -std::numbers::pi);
        declare_parameter("angle_max", std::numbers::pi);
        declare_parameter("range_min", 0.03);
        declare_parameter("range_max", 12.0);
        declare_parameter("invalid_range_is_inf", false);
        declare_parameter("enable_motor_pid", false);
        declare_parameter("target_scan_freq_hz", 10.0);

        // Read params
        frame_id_             = get_parameter("frame_id").as_string();
        angle_min_            = get_parameter("angle_min").as_double();
        angle_max_            = get_parameter("angle_max").as_double();
        range_min_            = get_parameter("range_min").as_double();
        range_max_            = get_parameter("range_max").as_double();
        invalid_range_is_inf_ = get_parameter("invalid_range_is_inf").as_bool();

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS());

        // Configure lidar
        tmini::Config cfg;
        cfg.port        = get_parameter("port").as_string();
        cfg.baud_rate   = static_cast<uint32_t>(get_parameter("baud_rate").as_int());
        cfg.motor_pid   = get_parameter("enable_motor_pid").as_bool();
        cfg.target_freq = static_cast<float>(get_parameter("target_scan_freq_hz").as_double());

        cfg.log_cb = [this](int level, const char* msg) {
            switch (level) {
                case tmini::kLogDebug: RCLCPP_INFO(get_logger(), "%s", msg); break;
                case tmini::kLogWarn:  RCLCPP_WARN(get_logger(), "%s", msg); break;
                default:               RCLCPP_ERROR(get_logger(), "%s", msg); break;
            }
        };

        lidar_ = std::make_unique<tmini::TminiLidar>(cfg);

        if (!lidar_->start([this](tmini::Scan&& scan) { on_scan(std::move(scan)); })) {
            RCLCPP_FATAL(get_logger(), "Failed to start TMini lidar on %s", cfg.port.c_str());
            throw std::runtime_error("Failed to start TMini lidar");
        }

        RCLCPP_INFO(get_logger(), "TMini lidar started (model=%d, tof=%s) on %s",
                     lidar_->model_code(),
                     lidar_->is_tof() ? "yes" : "no",
                     cfg.port.c_str());
    }

    ~TminiDriverNode() override {
        if (lidar_) lidar_->stop();
    }

private:
    void on_scan(tmini::Scan&& scan) {
        if (scan.samples.empty()) return;

        auto msg = sensor_msgs::msg::LaserScan();

        // Timestamp: convert steady_clock to ROS time
        // We capture the offset between steady_clock and system_clock once
        auto wall_start = std::chrono::system_clock::now() -
            (std::chrono::steady_clock::now() - scan.stamp_start);
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            wall_start.time_since_epoch()).count();
        msg.header.stamp.sec = static_cast<int32_t>(ns / 1'000'000'000LL);
        msg.header.stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000LL);
        msg.header.frame_id = frame_id_;

        // Scan timing
        auto scan_duration = std::chrono::duration<float>(
            scan.stamp_end - scan.stamp_start);
        float scan_time = scan_duration.count();

        msg.angle_min       = static_cast<float>(angle_min_);
        msg.angle_max       = static_cast<float>(angle_max_);
        msg.range_min       = static_cast<float>(range_min_);
        msg.range_max       = static_cast<float>(range_max_);
        msg.scan_time       = scan_time;

        // Fixed angular resolution: divide the FOV into bins
        float fov = static_cast<float>(angle_max_ - angle_min_);
        int num_bins = static_cast<int>(scan.samples.size());
        if (num_bins < 2) return;

        msg.angle_increment = fov / static_cast<float>(num_bins - 1);
        msg.time_increment  = scan_time / static_cast<float>(num_bins);

        // Initialize ranges
        float default_range = invalid_range_is_inf_
            ? std::numeric_limits<float>::infinity()
            : 0.0f;
        msg.ranges.assign(num_bins, default_range);
        msg.intensities.assign(num_bins, 0.0f);

        // Place each sample into the appropriate angular bin
        for (const auto& s : scan.samples) {
            // Convert lidar angle (0-360 deg) to ROS convention (-pi to pi rad)
            float angle_rad = s.angle_deg * (std::numbers::pi_v<float> / 180.0f);
            if (angle_rad > std::numbers::pi_v<float>)
                angle_rad -= 2.0f * std::numbers::pi_v<float>;

            if (angle_rad < angle_min_ || angle_rad > angle_max_) continue;

            int idx = static_cast<int>(
                std::round((angle_rad - angle_min_) / msg.angle_increment));
            if (idx < 0 || idx >= num_bins) continue;

            float range = s.distance_m;
            if (range < range_min_ || range > range_max_) {
                // Keep default (0 or inf)
                continue;
            }

            // If multiple samples land in the same bin, keep the closest
            if (msg.ranges[idx] == default_range || range < msg.ranges[idx]) {
                msg.ranges[idx] = range;
                msg.intensities[idx] = s.intensity;
            }
        }

        scan_pub_->publish(msg);
    }

    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    // Lidar
    std::unique_ptr<tmini::TminiLidar> lidar_;

    // Parameters
    std::string frame_id_;
    double angle_min_;
    double angle_max_;
    double range_min_;
    double range_max_;
    bool   invalid_range_is_inf_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(TminiDriverNode)

#ifndef COMPONENT_ONLY
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TminiDriverNode>(rclcpp::NodeOptions{});
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#endif

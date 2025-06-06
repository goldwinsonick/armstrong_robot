#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <map>
#include <string>
#include <vector>
#include <algorithm>

struct PIDConfig {
    double kp, ki, kd;
    double integral_clamp_min, integral_clamp_max;
    double output_min, output_max;
    std::string target_joint_name;
    std::string effort_command_topic;
    std::string target_angle_topic;
};

struct JointState {
    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;
    bool received = false;
};

class JointPIDController : public rclcpp::Node {
public:
    JointPIDController() : Node("joint_pid_controller_node") {
        double control_loop_rate = this->declare_parameter("control_loop_rate", 50.0);
        feedback_joint_states_topic_ = this->declare_parameter("feedback_joint_states_topic", "/joint_states");

        // Load joint configs
        load_joint_config("joint1");
        load_joint_config("joint2");

        // Subscribers
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            feedback_joint_states_topic_, 10,
            std::bind(&JointPIDController::joint_state_callback, this, std::placeholders::_1)
        );

        // Target angle subscribers and effort publishers for each joint
        for (const auto& [joint_name, config] : pid_configs_) {
            // Target angle subscriber
            auto sub = this->create_subscription<std_msgs::msg::Float64>(
                config.target_angle_topic, 10,
                [this, joint_name](const std_msgs::msg::Float64::SharedPtr msg) {
                    target_positions_[joint_name] = msg->data;
                    RCLCPP_INFO(this->get_logger(),
                        "Received target for %s: %.4f",
                        joint_name.c_str(), msg->data);
                }
            );
            target_subs_[joint_name] = sub;

            // Effort publisher (Float64MultiArray)
            auto pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                config.effort_command_topic, 10
            );
            effort_pubs_[joint_name] = pub;
        }

        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / control_loop_rate),
            std::bind(&JointPIDController::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Joint PID Controller started.");
    }

private:
    std::string feedback_joint_states_topic_;

    std::map<std::string, PIDConfig> pid_configs_;
    std::map<std::string, JointState> feedback_states_;
    std::map<std::string, double> target_positions_;
    std::map<std::string, double> integrals_;
    std::map<std::string, double> prev_errors_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr> effort_pubs_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> target_subs_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Time last_time_;

    void load_joint_config(const std::string& joint_param) {
        PIDConfig config;
        config.target_joint_name = this->declare_parameter(joint_param + ".target_joint_name", joint_param);
        config.effort_command_topic = this->declare_parameter(joint_param + ".effort_command_topic", "");
        config.target_angle_topic = this->declare_parameter(joint_param + ".target_angle_topic", "");
        config.kp = this->declare_parameter(joint_param + ".pid.kp", 0.0);
        config.ki = this->declare_parameter(joint_param + ".pid.ki", 0.0);
        config.kd = this->declare_parameter(joint_param + ".pid.kd", 0.0);
        config.integral_clamp_min = this->declare_parameter(joint_param + ".pid.integral_clamp_min", -100.0);
        config.integral_clamp_max = this->declare_parameter(joint_param + ".pid.integral_clamp_max", 100.0);
        config.output_min = this->declare_parameter(joint_param + ".output_limits.min", -100.0);
        config.output_max = this->declare_parameter(joint_param + ".output_limits.max", 100.0);
        pid_configs_[joint_param] = config;
        integrals_[joint_param] = 0.0;
        prev_errors_[joint_param] = 0.0;

        // Print out the loaded parameters for verification
        RCLCPP_INFO(this->get_logger(),
            "[%s] target_joint_name: %s, effort_command_topic: %s, target_angle_topic: %s",
            joint_param.c_str(), config.target_joint_name.c_str(),
            config.effort_command_topic.c_str(), config.target_angle_topic.c_str());
        RCLCPP_INFO(this->get_logger(),
            "[%s] kp: %.3f, ki: %.3f, kd: %.3f, i_clamp_min: %.3f, i_clamp_max: %.3f, out_min: %.3f, out_max: %.3f",
            joint_param.c_str(), config.kp, config.ki, config.kd,
            config.integral_clamp_min, config.integral_clamp_max,
            config.output_min, config.output_max);
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (const auto& [joint_param, config] : pid_configs_) {
            auto it = std::find(msg->name.begin(), msg->name.end(), config.target_joint_name);
            if (it != msg->name.end()) {
                size_t idx = std::distance(msg->name.begin(), it);
                feedback_states_[joint_param].position = msg->position[idx];
                if (msg->velocity.size() > idx)
                    feedback_states_[joint_param].velocity = msg->velocity[idx];
                if (msg->effort.size() > idx)
                    feedback_states_[joint_param].effort = msg->effort[idx];
                feedback_states_[joint_param].received = true;
            }
        }
    }

    void control_loop() {
        rclcpp::Time now = this->now();
        double dt = 0.02; // default to 20ms (50Hz)
        if (last_time_.nanoseconds() > 0) {
            dt = (now - last_time_).seconds();
        }
        last_time_ = now;

        for (const auto& [joint_param, config] : pid_configs_) {
            if (!feedback_states_[joint_param].received || target_positions_.find(joint_param) == target_positions_.end())
                continue;

            double pos = feedback_states_[joint_param].position;
            double target = target_positions_[joint_param];
            double error = target - pos;

            // PID calculations
            integrals_[joint_param] += error * dt;
            // Anti-windup
            integrals_[joint_param] = std::clamp(integrals_[joint_param], config.integral_clamp_min, config.integral_clamp_max);

            double derivative = (error - prev_errors_[joint_param]) / dt;
            prev_errors_[joint_param] = error;

            double effort = config.kp * error + config.ki * integrals_[joint_param] + config.kd * derivative;
            // Output limits
            effort = std::clamp(effort, config.output_min, config.output_max);

            // Publish effort as Float64MultiArray
            std_msgs::msg::Float64MultiArray effort_msg;
            effort_msg.data = {effort};
            effort_pubs_[joint_param]->publish(effort_msg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
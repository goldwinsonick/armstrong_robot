#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <string>
#include <vector>
#include <map>
#include <algorithm> // For std::clamp

// Simple PID state structure
struct PIDState {
    double integral = 0.0;
    double prev_error = 0.0;
    rclcpp::Time last_time;
    bool first_run = true;

    // Parameters
    double kp = 0.0, ki = 0.0, kd = 0.0;
    double integral_clamp_min = -1.0, integral_clamp_max = 1.0;
    double output_min = -1.0, output_max = 1.0;
};

class JointPIDControllerNode : public rclcpp::Node {
public:
    JointPIDControllerNode() : Node("joint_pid_controller_node") {
        RCLCPP_INFO(this->get_logger(), "Joint PID Controller Node starting up...");

        // Declare and get general parameters
        this->declare_parameter<double>("control_loop_rate", 50.0);
        this->declare_parameter<std::string>("feedback_joint_states_topic", "/joint_states");
        this->declare_parameter<std::string>("target_joint_angles_topic", "/arm/target_joint_states");

        control_loop_rate_ = this->get_parameter("control_loop_rate").as_double();
        std::string feedback_topic = this->get_parameter("feedback_joint_states_topic").as_string();
        std::string target_topic = this->get_parameter("target_joint_angles_topic").as_string();

        // Configure PID controllers for joint1 and joint2
        configure_joint_pid("joint1");
        configure_joint_pid("joint2");

        // Subscribers
        feedback_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            feedback_topic, rclpy::qos::qos_profile_sensor_data, // Use sensor data QoS for /joint_states
            std::bind(&JointPIDControllerNode::feedback_states_callback, this, std::placeholders::_1));
        
        target_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            target_topic, 10,
            std::bind(&JointPIDControllerNode::target_angles_callback, this, std::placeholders::_1));

        // Control loop timer
        if (control_loop_rate_ > 0) {
            auto timer_period = std::chrono::duration<double>(1.0 / control_loop_rate_);
            control_timer_ = this->create_wall_timer(
                timer_period, std::bind(&JointPIDControllerNode::control_loop_callback, this));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Control loop rate must be > 0.");
        }
        RCLCPP_INFO(this->get_logger(), "Node configured. Waiting for target and feedback states...");
    }

private:
    void configure_joint_pid(const std::string& joint_prefix) {
        this->declare_parameter<std::string>(joint_prefix + ".target_joint_name", joint_prefix);
        this->declare_parameter<std::string>(joint_prefix + ".effort_command_topic", "/" + joint_prefix + "_effort_controller/commands");
        this->declare_parameter<double>(joint_prefix + ".pid.kp", 0.0);
        this->declare_parameter<double>(joint_prefix + ".pid.ki", 0.0);
        this.declare_parameter<double>(joint_prefix + ".pid.kd", 0.0);
        this->declare_parameter<double>(joint_prefix + ".pid.integral_clamp_min", -1.0);
        this->declare_parameter<double>(joint_prefix + ".pid.integral_clamp_max", 1.0);
        this->declare_parameter<double>(joint_prefix + ".output_limits.min", -1.0);
        this->declare_parameter<double>(joint_prefix + ".output_limits.max", 1.0);

        std::string actual_joint_name = this->get_parameter(joint_prefix + ".target_joint_name").as_string();
        joint_params_[actual_joint_name].name = actual_joint_name;
        joint_params_[actual_joint_name].effort_topic = this->get_parameter(joint_prefix + ".effort_command_topic").as_string();
        joint_params_[actual_joint_name].pid_state.kp = this->get_parameter(joint_prefix + ".pid.kp").as_double();
        joint_params_[actual_joint_name].pid_state.ki = this->get_parameter(joint_prefix + ".pid.ki").as_double();
        joint_params_[actual_joint_name].pid_state.kd = this.get_parameter(joint_prefix + ".pid.kd").as_double();
        joint_params_[actual_joint_name].pid_state.integral_clamp_min = this->get_parameter(joint_prefix + ".pid.integral_clamp_min").as_double();
        joint_params_[actual_joint_name].pid_state.integral_clamp_max = this->get_parameter(joint_prefix + ".pid.integral_clamp_max").as_double();
        joint_params_[actual_joint_name].pid_state.output_min = this.get_parameter(joint_prefix + ".output_limits.min").as_double();
        joint_params_[actual_joint_name].pid_state.output_max = this.get_parameter(joint_prefix + ".output_limits.max").as_double();
        joint_params_[actual_joint_name].pid_state.last_time = this->get_clock()->now();


        joint_params_[actual_joint_name].publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            joint_params_[actual_joint_name].effort_topic, 10);
        
        RCLCPP_INFO(this->get_logger(), "Configured PID for joint: %s (Kp=%.2f, Ki=%.2f, Kd=%.2f) publishing to %s",
            actual_joint_name.c_str(),
            joint_params_[actual_joint_name].pid_state.kp,
            joint_params_[actual_joint_name].pid_state.ki,
            joint_params_[actual_joint_name].pid_state.kd,
            joint_params_[actual_joint_name].effort_topic.c_str());
    }

    struct JointParams {
        std::string name;
        std::string effort_topic;
        PIDState pid_state;
        double target_position = 0.0;
        double current_position = 0.0;
        double current_velocity = 0.0; // Not used in this PID version, but good to have
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    };

    std::map<std::string, JointParams> joint_params_;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double control_loop_rate_;


    void target_angles_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (joint_params_.count(msg->name[i])) {
                joint_params_[msg->name[i]].target_position = msg->position[i];
                 RCLCPP_DEBUG(this->get_logger(), "Received target for %s: %.2f", msg->name[i].c_str(), msg->position[i]);
            }
        }
    }

    void feedback_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (joint_params_.count(msg->name[i])) {
                joint_params_[msg->name[i]].current_position = msg.position[i];
                if (i < msg->velocity.size()) { // Check if velocity data is available
                    joint_params_[msg->name[i]].current_velocity = msg.velocity[i];
                }
                // RCLCPP_DEBUG(this->get_logger(), "Received feedback for %s: pos=%.2f, vel=%.2f", 
                //    msg->name[i].c_str(), msg->position[i], joint_params_[msg->name[i]].current_velocity);
            }
        }
    }

    void control_loop_callback() {
        rclcpp::Time current_time = this->get_clock()->now();

        for (auto& pair : joint_params_) {
            std::string joint_name = pair.first;
            JointParams& params = pair.second;
            PIDState& pid = params.pid_state;

            double dt_sec = 0.0;
            if (!pid.first_run) {
                dt_sec = (current_time - pid.last_time).seconds();
            }
            pid.last_time = current_time;

            if (dt_sec <= 0 && !pid.first_run) { // Avoid division by zero or stale dt if no new time
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "dt is zero or negative for joint %s, skipping PID update.", joint_name.c_str());
                // Re-publish previous effort or zero? For now, skip and publish nothing or last known.
                // To be safe, let's command zero if dt is bad after the first run
                if (!pid.first_run) {
                     std_msgs::msg::Float64MultiArray effort_msg;
                     effort_msg.data.push_back(0.0);
                     params.publisher->publish(effort_msg);
                }
                continue;
            }
            
            double error = params.target_position - params.current_position;

            // Proportional
            double p_term = pid.kp * error;

            // Integral (with anti-windup)
            pid.integral += pid.ki * error * dt_sec;
            pid.integral = std::clamp(pid.integral, pid.integral_clamp_min, pid.integral_clamp_max);
            double i_term = pid.integral;

            // Derivative
            double d_term = 0.0;
            if (!pid.first_run && dt_sec > 1e-6) { // Avoid division by zero on first run or too small dt
                d_term = pid.kd * (error - pid.prev_error) / dt_sec;
            }
            pid.prev_error = error;
            
            if (pid.first_run) {
                pid.first_run = false; // Derivative term will be zero for the first iteration
            }

            double output_effort = p_term + i_term + d_term;
            output_effort = std::clamp(output_effort, pid.output_min, pid.output_max);

            std_msgs::msg::Float64MultiArray effort_msg;
            effort_msg.data.push_back(output_effort);
            params.publisher->publish(effort_msg);

            RCLCPP_DEBUG(this->get_logger(), 
                "Joint: %s | Tgt: %.2f, Cur: %.2f, Err: %.2f | P: %.2f, I: %.2f, D: %.2f | Out: %.2f",
                joint_name.c_str(), params.target_position, params.current_position, error,
                p_term, i_term, d_term, output_effort);
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPIDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
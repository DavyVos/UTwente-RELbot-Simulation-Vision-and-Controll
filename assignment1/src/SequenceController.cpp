#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core/mat.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "relbot_msgs/msg/relbot_motors.hpp"
#include "assignment1/msg/object_position.hpp"
#include <tuple>
#include <iostream>

using namespace std::chrono_literals;
using namespace std;

const uint8_t setPointsSize = 7;
const tuple<double, double> setPoints[] = {
    tuple(1.0, 1.0),
    tuple(1.0, 2.0),
    tuple(1.0, 3.0),
    tuple(2.0, 4.0),
    tuple(3.0, 5.0),
    tuple(4.0, 5.0),
    tuple(5.0, 5.0),
};
const uint8_t setPointWheelVelSize = 7;
const double setPointWheelVel[] = {1, 2, 3, 4, 0, 5, 1};
const double initialSpeed = 1.0;
const double timeConstant = 1.0;

class SequenceController : public rclcpp::Node
{
public:
    SequenceController();
    ~SequenceController();

private:
    rclcpp::Publisher<relbot_msgs::msg::RelbotMotors>::SharedPtr publisher_;
    rclcpp::Subscription<assignment1::msg::ObjectPosition>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    void controllLoop();
    void setPointStrategy();
    void followObjectStrategy();
    void closedLoopFollowLight();
    int setPointsIterator_ = 0;
    tuple<double, double> relative2dTarget_ = tuple(0.0, 0.0);
    double theta_set_ = 0.0;
    double forward_set_ = 0.0;
    double est_y_error_ = 0.0;
    double est_x_error_ = 0.0;
    double est_theta_z_error = 0.0;
    tuple<double, double, double> currentObjectPosition_;
};

SequenceController::SequenceController() : Node("SequenceController")
{
    declare_parameter<string>("mode", "follow"); // Declare whether to follow or not
    this->publisher_ = this->create_publisher<relbot_msgs::msg::RelbotMotors>("/input/motor_cmd", 10);
    auto set_points_callback =
        [this]() -> void
    {
        controllLoop();
    };
    timer_ = this->create_wall_timer(1ms, set_points_callback);

    auto object_position_callback =
        [this](const assignment1::msg::ObjectPosition::SharedPtr msg) -> void
    {
        RCLCPP_INFO(this->get_logger(), "Received object position: %F, %F", msg->x, msg->y);
        this->currentObjectPosition_ = tuple<double, double, double>(msg->x, msg->y, msg->z);
        this->relative2dTarget_ = tuple<double, double>(msg->x, msg->z);
        const string mode = get_parameter("mode").as_string();
        const double distance = sqrt(((60 - msg->x) * (60 - msg->x)) + (msg->z * msg->z));
        // if (est_y_error_ < 5 && est_x_error_ < 5 && est_theta_z_error < CV_PI / 10)
        // {
        //     // est_x_error_ = (60 - msg->x) / distance;
        //     // est_y_error_ = distance;
        //     // est_theta_z_error = atan((60 - msg->x) / msg->z);
        //     double dx = 60.0 - msg->x;
        //     double dz = msg->z;

        //     est_theta_z_error = atan2(dx, dz);
        //     est_x_error_ = est_theta_z_error; // use angle directly
        //     // est_y_error_ = dz;                // forward error is depth
        //     // double dx = 60.0 - msg->x;
        //     // double dz = msg->z;

        //     const double desired_distance = 5.0;

        //     est_theta_z_error = atan2(dx, dz);
        //     est_y_error_ = dz;
        // }

        double dx = 60.0 - msg->x;
        double dz = msg->z;

        double measured_theta = atan2(dx, dz);
        double measured_y = dz;

        const double theta_tol = 0.02; // radians
        const double y_tol = 0.2;      // depth units

        bool theta_ready =
            est_theta_z_error > -theta_tol &&
            est_theta_z_error < theta_tol;

        bool y_ready =
            est_y_error_ > -y_tol &&
            est_y_error_ < y_tol;

        if (theta_ready && y_ready)
        {
            est_theta_z_error = measured_theta;
            est_x_error_ = measured_theta;
            est_y_error_ = measured_y;
        }

        // }
        // if (mode == "follow")
        //     followObjectStrategy();
        // else if (mode == "setpoints")
        //     setPointStrategy();
        // else if (mode == "followclosed")
        //     closedLoopFollowLight();
        // else
        //     setPointStrategy();
    };
    subscriber_ = this->create_subscription<assignment1::msg::ObjectPosition>("objectpos", 10, object_position_callback);
}

void SequenceController::followObjectStrategy()
{
    // x and y value between 0 and 100
    auto [x, y, z] = this->currentObjectPosition_;
    auto message = relbot_msgs::msg::RelbotMotors();
    // According to x deviation from center adjust left and right wheel velocity (max between 0 and 2.0)
    double horizontalDeviation = ((x - 50) / 100) * 5.0;
    message.left_wheel_vel = -(10.0 + horizontalDeviation);
    message.right_wheel_vel = 10.0 - horizontalDeviation;
    RCLCPP_INFO(this->get_logger(), "Adjusting to deviation: %F, left: %F, right %F", horizontalDeviation, message.left_wheel_vel, message.right_wheel_vel);
    this->publisher_->publish(message);
}

void SequenceController::closedLoopFollowLight()
{
    const double h = 0.001;
    const double tau = 10.0;
    // Desired values
    // dead center
    // auto [x, y] = this->relative2dTarget_;

    // est_y_error_ -= forward_set_ * h;

    // If robot rotates, relative x shifts
    // est_theta_z_error -= theta_set_ * h;
    est_y_error_ -= forward_set_ * h;
    est_theta_z_error -= (-2.0 * theta_set_) * h;
    // Integrate using forward Euler method
    double theta_dt = (1.0 / tau) * est_theta_z_error;
    double forward_dt = (1.0 / tau) * est_y_error_;

    theta_set_ += h * theta_dt;
    forward_set_ += h * forward_dt;

    auto message = relbot_msgs::msg::RelbotMotors();

    message.left_wheel_vel = -(forward_set_ + theta_set_);
    message.right_wheel_vel = (forward_set_ - theta_set_);
    // According to x deviation from center adjust left and right wheel velocity (max between 0 and 2.0)
    // RCLCPP_INFO(this->get_logger(), "Adjusting to deviation: %F, left: %F, right %F", horizontalDeviation, message.left_wheel_vel, message.right_wheel_vel);
    this->publisher_->publish(message);
}

void SequenceController::setPointStrategy()
{
    auto message = relbot_msgs::msg::RelbotMotors();
    double velocity = setPointWheelVel[setPointsIterator_];
    message.left_wheel_vel = -velocity;
    message.right_wheel_vel = velocity;
    RCLCPP_INFO(this->get_logger(), "Publishing: sequence point %F", message.left_wheel_vel);
    this->publisher_->publish(message);
    setPointsIterator_++;
    if (setPointsIterator_ > setPointsSize)
    {
        setPointsIterator_ = 0;
    }
}

void SequenceController::controllLoop()
{
    const double h = 0.001;
    const double tau = 0.5; // smaller = faster response

    est_y_error_ -= forward_set_ * h;
    est_theta_z_error -= theta_set_ * h;

    double theta_dot =
        (1.0 / tau) * est_theta_z_error - (2.0 / tau) * theta_set_;

    double forward_dot =
        (1.0 / tau) * est_y_error_ - (2.0 / tau) * forward_set_;

    theta_set_ += h * theta_dot;
    forward_set_ += h * forward_dot;

    // if (est_y_error_ < 0.1 && est_y_error_ > -0.1)
    //     forward_set_ = 0.0;

    // if (est_theta_z_error < 0.1 && est_theta_z_error > -0.1)
    //     theta_set_ = 0.0;

    auto message = relbot_msgs::msg::RelbotMotors();

    message.left_wheel_vel = -(forward_set_ - theta_set_);
    message.right_wheel_vel = (forward_set_ + theta_set_);

    publisher_->publish(message);
}

SequenceController::~SequenceController()
{
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<SequenceController>());
    rclcpp::shutdown();
    return 0;
}
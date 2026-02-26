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
#include "presets.h"

using namespace std::chrono_literals;
using namespace std;

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
    void controllLoop();
    void presetMode();
    void followMode();
    void toLocationMode();
    rclcpp::Publisher<relbot_msgs::msg::RelbotMotors>::SharedPtr mPublisher;
    rclcpp::Subscription<assignment1::msg::ObjectPosition>::SharedPtr mSubscriber;
    rclcpp::TimerBase::SharedPtr mTimer;
    int mSetPointsIterator = 0;
    double mThetaZ = 0.0;
    double mForward = 0.0;
    double mEstimatedYError = 0.0;
    double mEstimatedXError = 0.0;
    double mEstimatedThetaZError = 0.0;
    tuple<double, double, double> currentObjectPosition_;
};

SequenceController::SequenceController() : Node("SequenceController")
{
    // Modes: preset, follow, controlled.
    declare_parameter<string>("mode", "follow");

    // Closed loop controller parameters
    declare_parameter<double>("h", 0.001);
    declare_parameter<double>("tau", 10.0);
    declare_parameter<uint8_t>("preset", 0);

    this->mPublisher = this->create_publisher<relbot_msgs::msg::RelbotMotors>("/input/motor_cmd", 10);
    auto set_points_callback =
        [this]() -> void {
            this->controllLoop();
        };
    mTimer = this->create_wall_timer(1ms, set_points_callback);

    auto object_position_callback =
        [this](const assignment1::msg::ObjectPosition::SharedPtr msg) -> void
    {
        RCLCPP_INFO(this->get_logger(), "Received object position: %F, %F", msg->x, msg->y);
        this->currentObjectPosition_ = tuple<double, double, double>(msg->x, msg->y, msg->z);
        const string mode = get_parameter("mode").as_string();
        const double distance = sqrt(((60 - msg->x) * (60 - msg->x)) + (msg->z * msg->z));

        const double h = get_parameter("h").as_double();
        const double tau = get_parameter("tau").as_double();

        // normalize camera input
        double dx = 60.0 - msg->x;
        double dz = msg->z;

        double measuredThetaZError = atan2(dx, dz);
        double measuredForwardError = dz;

        const double thetaTolerance = 0.02;  // radians
        const double forwardTolerance = 0.2; // arbitrary depth units

        if (mEstimatedThetaZError > -thetaTolerance &&
            mEstimatedThetaZError < thetaTolerance)
        {
            mEstimatedThetaZError = measuredThetaZError;
        }
        if (mEstimatedYError > -forwardTolerance &&
            mEstimatedYError < forwardTolerance)
        {
            mEstimatedYError = measuredForwardError;
        }
    };
    mSubscriber = this->create_subscription<assignment1::msg::ObjectPosition>("objectpos", 10, object_position_callback);
}

void SequenceController::followMode()
{
    auto [x, y, z] = this->currentObjectPosition_;
    auto message = relbot_msgs::msg::RelbotMotors();
    // According to x deviation from center adjust left and right wheel velocity (max between 0 and 2.0)
    message.left_wheel_vel = -(mForward - mThetaZ);
    message.right_wheel_vel = (mForward + mThetaZ);
    RCLCPP_INFO(this->get_logger(), "Left wheel_vel: %F, Right wheel_vel: %F", message.left_wheel_vel, message.right_wheel_vel);
    this->mPublisher->publish(message);
}

void SequenceController::toLocationMode()
{
    const double h = get_parameter("h").as_double();     // (was 0.001)
    const double tau = get_parameter("tau").as_double(); // smaller = faster response (was 0.5)

    mEstimatedYError -= mForward * h;
    mEstimatedThetaZError -= mThetaZ * h;

    double theta_dot = (1.0 / tau) * mEstimatedThetaZError - (2.0 / tau) * mThetaZ;

    double forward_dot = (1.0 / tau) * mEstimatedYError - (2.0 / tau) * mForward;

    mThetaZ += h * theta_dot;
    mForward += h * forward_dot;

    auto message = relbot_msgs::msg::RelbotMotors();

    message.left_wheel_vel = -(mForward - mThetaZ);
    message.right_wheel_vel = (mForward + mThetaZ);

    mPublisher->publish(message);
}

void SequenceController::presetMode()
{
    auto message = relbot_msgs::msg::RelbotMotors();
    double velocity = setPointWheelVel[mSetPointsIterator];
    message.left_wheel_vel = -velocity;
    message.right_wheel_vel = velocity;
    RCLCPP_INFO(this->get_logger(), "Publishing: sequence point %F", message.left_wheel_vel);
    this->mPublisher->publish(message);
    mSetPointsIterator++;
    if (mSetPointsIterator > 4)
    {
        mSetPointsIterator = 0;
    }
}

void SequenceController::controllLoop()
{
    string mode = get_parameter("mode").as_string();
    if (mode == "follow")
        followMode();
    else if (mode == "preset")
        presetMode();
    else if (mode == "controlled")
        toLocationMode();
    else
        presetMode();
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
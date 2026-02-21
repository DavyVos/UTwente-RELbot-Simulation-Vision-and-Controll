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
#include "assignment1/msg/object_brightness.hpp"

using namespace std::chrono_literals;

const int totalPixels = 76800;

class DetectBrightness : public rclcpp::Node
{
public:
    DetectBrightness()
        : Node("DetectBrightness")
    {
        declare_parameter<float>("threshold", 80.0);
        publisher_ = this->create_publisher<assignment1::msg::ObjectBrightness>("brightness", 10);
        auto timer_callback =
            [this]() -> void
        {
            auto message = assignment1::msg::ObjectBrightness();
            message.brightness = brightness_;
            if (isDark)
            {
                message.dark = true;
                RCLCPP_INFO(this->get_logger(), "Average light value %f, It is dark", message.brightness);
            }
            else
            {
                message.dark = false;
                RCLCPP_INFO(this->get_logger(), "Average light value %f, It is light", message.brightness);
            }
            this->publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);

        auto image_callback =
            [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
        {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            // https://stackoverflow.com/questions/10344246/how-can-i-convert-a-cvmat-to-a-gray-scale-in-opencv
            cv::Mat grayImage;
            cvtColor(image, grayImage, CV_BGR2GRAY);
            cv::imshow("view", image);
            cv::waitKey(1);
            int totalLight = 0;
            // https://stackoverflow.com/questions/4504687/cycle-through-pixels-with-opencv
            for (int i = 0; i < image.rows; i++)
                for (int j = 0; j < grayImage.cols; j++)
                    totalLight += grayImage.at<uchar>(i, j);

            double avgLightPP = (double)totalLight / (double)totalPixels;
            brightness_ = avgLightPP;
            this->isDark = avgLightPP < get_parameter("threshold").as_double();

            if (isDark)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Received image with width: %u, height: %u, Average light amount: %f, it is: DARK",
                            msg->width, msg->height, avgLightPP);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),
                            "Received image with width: %u, height: %u, Average light amount: %f, it is: LIGHT",
                            msg->width, msg->height, avgLightPP);
            }
        };

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image", // topic name
            10,
            image_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<assignment1::msg::ObjectBrightness>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    size_t count_;
    double brightness_;
    bool isDark;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectBrightness>());
    rclcpp::shutdown();
    return 0;
}
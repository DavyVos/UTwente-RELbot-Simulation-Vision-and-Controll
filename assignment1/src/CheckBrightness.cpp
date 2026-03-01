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

class DetectBrightness : public rclcpp::Node
{
public:
    DetectBrightness()
        : Node("DetectBrightness")
    {
        declare_parameter<float>("threshold", 80.0);    //Set the theshold for 
        declare_parameter<bool>("use_preview", false);  //Display a preview window of what the image processor sees
        mPublisher = this->create_publisher<assignment1::msg::ObjectBrightness>("brightness", 10);
        auto timer_callback =
            [this]() -> void
        {
            auto message = assignment1::msg::ObjectBrightness();
            message.brightness = mBrightness;
            if (mIsDark)
            {
                message.dark = true;
                RCLCPP_INFO(this->get_logger(), "Average light value %f, It is dark", message.brightness);
            }
            else
            {
                message.dark = false;
                RCLCPP_INFO(this->get_logger(), "Average light value %f, It is light", message.brightness);
            }
            this->mPublisher->publish(message);
        };
        mTimer = this->create_wall_timer(100ms, timer_callback);

        auto image_callback =
            [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
        {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            // https://stackoverflow.com/questions/10344246/how-can-i-convert-a-cvmat-to-a-gray-scale-in-opencv
            cv::Mat grayImage;
            cvtColor(image, grayImage, CV_BGR2GRAY);
            int totalLight = 0;
            // https://stackoverflow.com/questions/4504687/cycle-through-pixels-with-opencv
            for (int i = 0; i < image.rows; i++)
            {
                for (int j = 0; j < grayImage.cols; j++)
                {
                    totalLight += grayImage.at<uchar>(i, j);
                }
            }

            double totalCameraPixels = msg->width * msg->height;
            double avgLightPP = (double)totalLight / totalCameraPixels;
            mBrightness = avgLightPP;
            this->mIsDark = avgLightPP < get_parameter("threshold").as_double();

            if (mIsDark)
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

        mSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            "image", // topic name
            10,
            image_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Publisher<assignment1::msg::ObjectBrightness>::SharedPtr mPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mSubscription;
    size_t mCount;
    double mBrightness;
    bool mIsDark;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectBrightness>());
    rclcpp::shutdown();
    return 0;
}
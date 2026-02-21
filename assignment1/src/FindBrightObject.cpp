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
#include "assignment1/msg/object_position.hpp"

using namespace std::chrono_literals;
using namespace std;

int threshold_type = 3;
int const max_value = 255;
int const max_type = 4;
int const max_binary_value = 255;

class FindBrightObject : public rclcpp::Node
{
public:
    FindBrightObject()
        : Node("FindObject")
    {
        declare_parameter<int>("threshold", 80);
        declare_parameter<string>("method", "basic");                       // Define methods "basic", "hsvcircle"
        declare_parameter<std::vector<long>>("hsv_lower", {90, 150, 140});  // set HSV method lower bound
        declare_parameter<std::vector<long>>("hsv_upper", {102, 255, 240}); // setHSV upper bound
        publisher_ = this->create_publisher<assignment1::msg::ObjectPosition>("objectpos", 10);
        auto timer_callback =
            [this]() -> void
        {
            auto message = assignment1::msg::ObjectPosition();
            message.x = this->horizontalCenter_;
            message.y = this->verticalCenter_;
            message.z = this->distance_;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%F, %F", message.x, message.y);
            this->publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(10ms, timer_callback);

        auto image_callback =
            [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
        {
            string method = get_parameter("method").as_string();

            if (method == "basic")
            {
                this->findBright(msg);
            }
            else if (method == "hsvcircle")
            {
                this->findHSV(msg);
            }
        };

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image",
            10,
            image_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<assignment1::msg::ObjectPosition>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    void findBright(sensor_msgs::msg::Image::SharedPtr img);
    void findHSV(sensor_msgs::msg::Image::SharedPtr img);
    size_t count_;
    double horizontalCenter_;
    double verticalCenter_;
    double distance_;
};

void FindBrightObject::findBright(sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    // https://stackoverflow.com/questions/10344246/how-can-i-convert-a-cvmat-to-a-gray-scale-in-opencv
    cv::Mat grayImage, mask;
    cvtColor(image, grayImage, CV_BGR2GRAY);

    int horizontalWeight = 0;
    int verticalWeight = 0;
    int countWhitePixels = 0;
    int threshold_value = get_parameter("threshold").as_int();
    // https://docs.opencv.org/4.x/db/d8e/tutorial_threshold.html
    cv::threshold(grayImage, mask, threshold_value, max_binary_value, threshold_type);
    for (int i = 0; i < mask.rows; i++)
    {
        for (int j = 0; j < mask.cols; j++)
        {
            int pixelValue = (int)mask.at<uchar>(i, j);
            if (pixelValue == max_value)
            {
                horizontalWeight += j;
                verticalWeight += i;
                countWhitePixels++;
            }
        }
    }

    if (countWhitePixels == 0)
    {
        this->horizontalCenter_ = msg->width / 2;
        this->verticalCenter_ = msg->height / 2;
        this->distance_ = 0;
    }
    else
    {
        this->horizontalCenter_ = ((double)horizontalWeight / (double)countWhitePixels);
        this->verticalCenter_ = ((double)verticalWeight / (double)countWhitePixels);
        double totalPixels = msg->width * msg->height;
        this->distance_ = ((double)countWhitePixels / totalPixels) * 100.0;
    }

    int thickness = 2;
    int lineType = 8;
    cv::ellipse(cv_bridge::toCvShare(msg, "bgr8")->image,
                cv::Point(this->horizontalCenter_ * msg->width / 100.0, this->verticalCenter_ * msg->height / 100.0),
                cv::Size(5, 5),
                0,
                0,
                360,
                cv::Scalar(255, 0, 0),
                thickness,
                lineType);
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(1);
}

void FindBrightObject::findHSV(sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat toHSV;
    cv::cvtColor(img, toHSV, cv::COLOR_BGR2HSV);
    // https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
    // Billy cup HSV 192, 91%, 75%  // halve the hue (360 to 180 range) -> 96
    auto lower = get_parameter("hsv_lower").as_integer_array();
    auto upper = get_parameter("hsv_upper").as_integer_array();
    cv::Scalar lower_bound(lower[0], lower[1], lower[2]);
    cv::Scalar upper_bound(upper[0], upper[1], upper[2]);

    // Threshold the HSV image to get only blue colors
    // mask = cv.inRange(hsv, lower_blue, upper_blue)
    cv::Mat mask;
    cv::inRange(toHSV, lower_bound, upper_bound, mask);

    // Remove white dots and black dots from the mask to prepare for contours
    cv::Mat cleaned;
    cv::morphologyEx(mask, cleaned, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, {5, 5}));
    cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, {7, 7}));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cleaned, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestCircularity = 0.0;
    int bestIndex = -1;
    // Find the most circle like circle
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        double perimeter = cv::arcLength(contours[i], true);

        if (perimeter == 0)
            continue;

        double circularity = 4 * CV_PI * area / (perimeter * perimeter);

        if (area < 50)
            continue;

        if (circularity > bestCircularity)
        {
            bestCircularity = circularity;
            bestIndex = i;
        }
    }

    // Check for if not found
    cv::Point2f center;
    float radius;
    if (bestIndex >= 0)
    {
        cv::minEnclosingCircle(contours[bestIndex], center, radius);
        //y=28.8-0.32x rough  distance estimation using linearization from radius
        distance_ = 28.8-0.32 * radius;
        horizontalCenter_ = center.x;
        verticalCenter_ = center.y;
    }
    else
    {
        this->horizontalCenter_ = msg->width / 2;
        this->verticalCenter_ = msg->height / 2;
        this->distance_ = 0;
        center = cv::Point(horizontalCenter_, verticalCenter_);
        radius = 5;
    }
    cv::circle(img, center, (int)radius, cv::Scalar(0, 255, 0), 2);

    int thickness = 2;
    int lineType = 8;
    cv::imshow("view", img);
    cv::waitKey(1);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindBrightObject>());
    rclcpp::shutdown();
    return 0;
}
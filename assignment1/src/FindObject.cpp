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
        declare_parameter<string>("method", "brightness");                  // Define methods "brightness", "hsvcircle"
        declare_parameter<std::vector<long>>("hsv_lower", {90, 150, 140});  // set HSV method lower bound
        declare_parameter<std::vector<long>>("hsv_upper", {102, 255, 240}); // setHSV upper bound
        declare_parameter<bool>("use_preview", false);                      // Display a preview window of what the image processor sees
        declare_parameter<double>("circularity", 0.0);                      // Threshold for how round the hsvcircle should be
        mPublisher = this->create_publisher<assignment1::msg::ObjectPosition>("objectpos", 10);
        auto timer_callback =
            [this]() -> void
        {
            auto message = assignment1::msg::ObjectPosition();
            message.x = this->mHorizontalCenter;
            message.y = this->mVerticalCenter;
            message.z = this->mDistance;
            this->mPublisher->publish(message);
        };
        mTimer = this->create_wall_timer(10ms, timer_callback);

        auto image_callback =
            [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
        {
            string method = get_parameter("method").as_string();

            if (method == "brightness")
            {
                this->findBright(msg);
            }
            else if (method == "hsvcircle")
            {
                this->findHSV(msg);
            }
        };

        mSubscription = this->create_subscription<sensor_msgs::msg::Image>(
            "image",
            10,
            image_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Publisher<assignment1::msg::ObjectPosition>::SharedPtr mPublisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mSubscription;
    void findBright(sensor_msgs::msg::Image::SharedPtr img);
    void findHSV(sensor_msgs::msg::Image::SharedPtr img);
    size_t mCount;
    double mHorizontalCenter;
    double mVerticalCenter;
    double mDistance;
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
    cv::threshold(grayImage, mask, threshold_value, max_binary_value, CV_THRESH_BINARY);
    for (int i = 0; i < mask.rows; i++)
    {
        for (int j = 0; j < mask.cols; j++)
        {
            int pixelValue = (int)mask.at<uchar>(i, j);
            if (pixelValue > 0)
            {
                horizontalWeight += j;
                verticalWeight += i;
                countWhitePixels++;
            }
        }
    }

    if (countWhitePixels == 0)
    {
        this->mHorizontalCenter = msg->width / 2;
        this->mVerticalCenter = msg->height / 2;
        this->mDistance = 0;
    }
    else
    {
        this->mHorizontalCenter = ((double)horizontalWeight / (double)countWhitePixels);
        this->mVerticalCenter = ((double)verticalWeight / (double)countWhitePixels);
        double totalPixels = msg->width * msg->height;
        this->mDistance = ((double)countWhitePixels / totalPixels) * 100.0;
    }

    int thickness = 2;
    int lineType = 8;
    cv::ellipse(cv_bridge::toCvShare(msg, "bgr8")->image,
                cv::Point(this->mHorizontalCenter * msg->width / 100.0, this->mVerticalCenter * msg->height / 100.0),
                cv::Size(5, 5),
                0,
                0,
                360,
                cv::Scalar(255, 0, 0),
                thickness,
                lineType);
    bool usePreview = get_parameter("use_preview").as_bool();
    if (usePreview)
    {
        RCLCPP_INFO(this->get_logger(), "object info: x=%d, y=%d", horizontalWeight, verticalWeight);
        cv::imshow("view", mask);
        cv::waitKey(1);
    }
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

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double circularityThreshold = get_parameter("circularity").as_double();
    double bestCircularity = 0.0;
    int bestIndex = -1;
    // Find the most circle like circle
    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        double perimeter = cv::arcLength(contours[i], true);

        if (perimeter == 0)
        {
            continue;
        }

        double expectedArea = perimeter * perimeter;
        double areaPerimeterRatio = area / expectedArea;
        if (area < 50)
        {
            continue;
        }

        if (areaPerimeterRatio > bestCircularity && areaPerimeterRatio > circularityThreshold)
        {
            bestCircularity = areaPerimeterRatio;
            bestIndex = i;
        }
    }

    // Check for if not found
    cv::Point2f center;
    float radius;
    if (bestIndex >= 0)
    {
        cv::minEnclosingCircle(contours[bestIndex], center, radius);
        // y=28.8-0.32x rough  distance estimation using linearization from radius
        mDistance = 28.8 - 0.32 * radius;
        mHorizontalCenter = center.x;
        mVerticalCenter = center.y;
    }
    else
    {
        mHorizontalCenter = msg->width / 2;
        mVerticalCenter = msg->height / 2;
        mDistance = 0;
        center = cv::Point(mHorizontalCenter, mVerticalCenter);
        radius = 5;
    }
    cv::circle(img, center, (int)radius, cv::Scalar(0, 255, 0), 2);
    cv::circle(mask, center, (int)radius, cv::Scalar(0, 255, 0), 2);

    bool usePreview = get_parameter("use_preview").as_bool();
    if (usePreview)
    {
        RCLCPP_INFO(this->get_logger(), "object info: found=%d circularity=%F, x=%F, y=%F",
                    bestIndex > 0,
                    bestCircularity,
                    mHorizontalCenter,
                    mVerticalCenter);
        cv::imshow("view", img);
        cv::waitKey(1);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FindBrightObject>());
    rclcpp::shutdown();
    return 0;
}
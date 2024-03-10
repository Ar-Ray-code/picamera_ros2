#include "picamera_ros2/picamera_pub.hpp"

namespace picamera_ros
{

PiCameraROS::PiCameraROS(const rclcpp::NodeOptions &options_): Node("picamera_ros", options_)
{
    this->camera_ = new lccv::PiCamera();

    this->declare_parameter("video_width", 1280);
    this->declare_parameter("video_height", 720);
    this->declare_parameter("framerate", 30);
    this->declare_parameter("hdr", true);
    this->declare_parameter("verbose", false);
    this->declare_parameter("shutter", 100.0);

    this->get_parameter("video_width", this->video_width_);
    this->get_parameter("video_height", this->video_height_);
    this->get_parameter("framerate", this->framerate_);
    this->get_parameter("hdr", this->hdr_);
    this->get_parameter("verbose", this->verbose_);
    this->get_parameter("shutter", this->shutter_);

    this->camera_->options->video_width = this->video_width_;
    this->camera_->options->video_height = this->video_height_;
    this->camera_->options->framerate = this->framerate_;
    this->camera_->options->verbose = this->verbose_;
    this->camera_->options->shutter = this->shutter_;

    this->camera_->hdrOpen(this->hdr_);
    this->camera_->startVideo();

    this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / this->framerate_), std::bind(&PiCameraROS::timerCallback, this));
}

PiCameraROS::~PiCameraROS()
{
    this->camera_->stopVideo();
}

void PiCameraROS::timerCallback()
{
    sensor_msgs::msg::Image image_msg;
    cv_bridge::CvImage cv_image;
    cv::Mat image;

    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    this->camera_->getVideoFrame(image, 1000);
    cv_image.image = image;
    cv_image.toImageMsg(image_msg);
    image_pub_->publish(image_msg);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(picamera_ros::PiCameraROS)

int main(int argc, char** argv)
{
	using namespace picamera_ros;
    using namespace lccv;
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	auto node = std::make_shared<PiCameraROS>(options);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
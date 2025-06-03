#include "picamera_ros2/picamera_pub.hpp"

namespace picamera_ros
{

PiCameraROS::PiCameraROS(const rclcpp::NodeOptions &options_): Node("picamera_ros", options_)
{
    this->camera_ = new lccv::PiCamera();

    this->param_listener_ = std::make_shared<picamera_parameters::ParamListener>(
        this->get_node_parameters_interface());
    this->params_ = param_listener_->get_params();

    std::map<std::string, int> exposure_table =
		{ { "normal", libcamera::controls::ExposureNormal },
        { "sport", libcamera::controls::ExposureShort },
        { "long", libcamera::controls::ExposureLong },
        { "custom", libcamera::controls::ExposureCustom } };
    std::map<std::string, int> awb_table =
        { { "auto", libcamera::controls::AwbAuto },
        { "incandescent", libcamera::controls::AwbIncandescent },
        { "tungsten", libcamera::controls::AwbTungsten },
        { "fluorescent", libcamera::controls::AwbFluorescent },
        { "indoor", libcamera::controls::AwbIndoor },
        { "daylight", libcamera::controls::AwbDaylight },
        { "cloudy", libcamera::controls::AwbCloudy },
        { "custom", libcamera::controls::AwbCustom } };
    std::map<std::string, int> metering_table =
		{ { "centre", libcamera::controls::MeteringCentreWeighted },
        { "spot", libcamera::controls::MeteringSpot },
        { "average", libcamera::controls::MeteringMatrix },
        { "matrix", libcamera::controls::MeteringMatrix },
        { "custom", libcamera::controls::MeteringCustom } };
	std::map<std::string, int> afMode_table =
		{ { "default", -1 },
        { "manual", libcamera::controls::AfModeManual },
        { "auto", libcamera::controls::AfModeAuto },
        { "continuous", libcamera::controls::AfModeContinuous } };
	std::map<std::string, int> afRange_table =
        { { "normal", libcamera::controls::AfRangeNormal },
        { "macro", libcamera::controls::AfRangeMacro },
        { "full", libcamera::controls::AfRangeFull } };
    std::map<std::string, int> afSpeed_table =
        { { "normal", libcamera::controls::AfSpeedNormal },
        { "fast", libcamera::controls::AfSpeedFast } };

    this->camera_->options->video_width = this->params_.video_width;
    this->camera_->options->video_height = this->params_.video_height;
    this->camera_->options->photo_width = this->params_.camera_width;
    this->camera_->options->photo_height = this->params_.camera_height;
    this->camera_->options->camera = this->params_.id;
    this->camera_->options->shutter = this->params_.shutter;
    this->camera_->options->denoise = this->params_.denoise;
    this->camera_->options->brightness = this->params_.brightness;
    this->camera_->options->saturation = this->params_.saturation;
    this->camera_->options->ev = this->params_.ev;
    this->camera_->options->lens_position = this->params_.lens_position;
    this->camera_->options->sharpness = this->params_.sharpness;
    this->camera_->options->contrast = this->params_.contrast;
    this->camera_->options->framerate = this->params_.framerate;
    this->camera_->options->setMetering(static_cast<Metering_Modes>(metering_table[this->params_.metering]));
    this->camera_->options->setExposureMode(static_cast<Exposure_Modes>(exposure_table[this->params_.exposure]));
    this->camera_->options->setWhiteBalance(static_cast<WhiteBalance_Modes>(awb_table[this->params_.awb]));
    this->camera_->options->setAfMode(static_cast<AfMode_Modes>(afMode_table[this->params_.afmode]));
    this->camera_->options->setAfRange(static_cast<AfRange_Modes>(afRange_table[this->params_.afrange]));
    this->camera_->options->setAfSpeed(static_cast<AfSpeed_Modes>(afSpeed_table[this->params_.afspeed]));

    this->camera_->options->framerate = (float)this->params_.framerate;

    this->camera_->hdrOpen(this->params_.hdr);
    this->camera_->startVideo();

    this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / this->params_.framerate), std::bind(&PiCameraROS::timerCallback, this));
}

PiCameraROS::~PiCameraROS()
{
    this->camera_->stopVideo();
}

void PiCameraROS::timerCallback()
{
    sensor_msgs::msg::Image image_msg;
    cv::Mat image;

    this->camera_->getVideoFrame(image, 1000);
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to capture image from camera.");
        return;
    }
    
    // Convert OpenCV Mat to ROS Image message
    matToImageMsg(image, image_msg, sensor_msgs::image_encodings::BGR8);
    image_pub_->publish(image_msg);
}

void PiCameraROS::matToImageMsg(const cv::Mat& image, sensor_msgs::msg::Image& ros_image, const std::string& encoding)
{
    ros_image.header.stamp = this->now();
    ros_image.header.frame_id = "camera";  // Add frame_id for proper TF integration
    ros_image.height = image.rows;
    ros_image.width = image.cols;
    ros_image.encoding = encoding;
    ros_image.is_bigendian = false;  // Most systems are little-endian
    ros_image.step = image.cols * image.elemSize();
    size_t size = ros_image.step * image.rows;
    ros_image.data.resize(size);

    if (image.isContinuous()) {
        memcpy(ros_image.data.data(), image.data, size);
    } else {
        // Copy by row by row for non-continuous images
        for (int i = 0; i < image.rows; ++i) {
            memcpy(ros_image.data.data() + i * ros_image.step, image.ptr(i), ros_image.step);
        }
    }
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
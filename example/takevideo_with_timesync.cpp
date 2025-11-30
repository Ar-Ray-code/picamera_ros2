#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

// host steady clock helper (matches timesync.hpp)
static inline int64_t now_steady_ns()
{
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

int main()
{
    std::cout << "Sample program for LCCV video capture with TimeSync\n";
    std::cout << "Press ESC to stop.\n";

    cv::Mat image;
    lccv::PiCamera cam;

    cam.options->video_width  = 1024;
    cam.options->video_height = 768;
    cam.options->framerate    = 5;
    cam.options->verbose      = true;

    cv::namedWindow("Video", cv::WINDOW_NORMAL);
    cam.startVideo();

    int ch = 0;
    while (ch != 27)
    {
        if (!cam.getVideoFrame(image, 1000)) {
            std::cout << "Timeout error\n";
            continue;
        }

        // ---- SHOW TIMING INFO ----
        const bool    ready       = cam.tsync_.ready();
        const int64_t H_exp       = cam.tsync_.exposureHostSteadyNs();
        const int64_t offset_ns   = cam.tsync_.offsetSensorToHostNs();
        const int64_t mad_ns      = cam.tsync_.jitterMadNs();

        std::cout << "Frame:"
                  << " exp(ns)=" << H_exp
                  << " | ready=" << ready
                  << " | offset(ms)=" << offset_ns * 1e-6
                  << " | jitter MAD(ms)=" << mad_ns * 1e-6
                  << std::endl;

        // (OPTIONAL) Convert host-steady exposure → system/ROS time
        int64_t R_exp = cam.tsync_.exposureRosNs(
            /* ros_now_ns */ [](){
                return std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
            },
            /* steady_now_ns */ [](){ return now_steady_ns(); }
        );
        std::cout << "   Converted to system_clock: " << R_exp << " ns\n";

        // ---- SHOW VIDEO ----
        cv::imshow("Video", image);
        ch = cv::waitKey(10);
    }

    cam.stopVideo();
    cv::destroyWindow("Video");
    return 0;
}

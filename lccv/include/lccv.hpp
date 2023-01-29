#ifndef LCCV_HPP
#define LCCV_HPP

#include <mutex>
#include <atomic>
#include <pthread.h>
#include <opencv2/opencv.hpp>

#include "libcamera_app.hpp"
#include "libcamera_app_options.hpp"

// HDR implementation ==============================================
// https://github.com/raspberrypi/libcamera-apps/commit/4fea2eed68300dcc88e89aa30da6079d10dce822

#include <fcntl.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/unistd.h>

static int xioctl(int fd, unsigned long ctl, void *arg)
{
	int ret, num_tries = 10;
	do
	{
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}
// ==================================================================

namespace lccv {

class PiCamera {
public:
    PiCamera();
    ~PiCamera();

    Options *options;

    //Photo mode
    bool startPhoto();
    bool capturePhoto(cv::Mat &frame);
    bool stopPhoto();

    //Video mode
    bool startVideo();
    bool getVideoFrame(cv::Mat &frame, unsigned int timeout);
    void stopVideo();

    bool hdrOpen(bool _hdr_flag);

protected:
    void run();
protected:
    LibcameraApp *app;
    void getImage(cv::Mat &frame, CompletedRequestPtr &payload);
    static void *videoThreadFunc(void *p);
    pthread_t videothread;
    unsigned int still_flags;
    unsigned int vw,vh,vstr;
    std::atomic<bool> running,frameready;
    uint8_t *framebuffer;
    std::mutex mtx;
    bool camerastarted;
};

}
#endif

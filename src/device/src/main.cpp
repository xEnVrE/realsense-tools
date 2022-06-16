#include <cstdlib>

#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <yarp/cv/Cv.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Image.h>

using namespace yarp::cv;
using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char ** argv)
{
    /* Storage for rgb, depth and time stamps. */
    std::vector<cv::Mat> rgb_frames;
    std::vector<cv::Mat> depth_frames;
    std::vector<double> stamps;

    /* Storage for output. */
    cv::Mat output_rgb;
    cv::Mat output_depth;

    /* YARP network. */
    Network yarp_network;
    if (!yarp_network.checkNetwork())
        throw(std::runtime_error("Cannot find the YARP network. Is the yarp server running?"));

    /* YARP output ports. */
    BufferedPort<ImageOf<PixelRgb>> yarp_output_rgb;
    BufferedPort<ImageOf<PixelFloat>> yarp_output_depth;
    yarp_output_rgb.open("/depthCamera/rgbImage:o");
    yarp_output_depth.open("/depthCamera/depthImage:o");

    /* YARP stamps. */
    Stamp yarp_stamp;

    /* Create pipeline. */
    std::cout << "Waiting for the camera to be ready..." << std::flush;
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 60);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 60);
    pipe.start(cfg);
    std::cout << " OK." << std::endl;

    /* Create instance of alignment procedure. */
    rs2::align align_to_color(RS2_STREAM_COLOR);

    /* While loop for data collection. */
    rs2::colorizer c;
    while (true)
    {
        /* Wait for new frames. */
        rs2::frameset frameset = pipe.wait_for_frames();

        /* Get current time. */
        auto now = std::chrono::steady_clock::now();
        auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        auto epoch_ns = now_ns.time_since_epoch();
        double absolute_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch_ns).count() / (1000.0 * 1000.0 * 1000.0);

        /* Align frames. */
        frameset = align_to_color.process(frameset);

        /* Extract separate frames and colorize. */
        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();
        /* Not sure it is required, however it stabilizes frame rate! */
        auto colorized_depth = c.colorize(depth);

        /* Wrap data around OpenCV matrices. */
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        cv::Mat cv_rgb(cv::Size(w, h), CV_8UC3, (void*) color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat cv_depth(cv::Size(w, h), CV_16U, (void*) depth.get_data(), cv::Mat::AUTO_STEP);
        cv_depth.convertTo(cv_depth, CV_32FC1, 0.001);

        output_rgb = cv_rgb.clone();
        cv::cvtColor(output_rgb, output_rgb, cv::COLOR_RGB2BGR);
        output_depth = cv_depth.clone();

        /* Set time stamp. */
        yarp_stamp.update();
        yarp_output_rgb.setEnvelope(yarp_stamp);
        yarp_output_depth.setEnvelope(yarp_stamp);

        /* Prepare images. */
        ImageOf<PixelRgb>& yarp_rgb_image = yarp_output_rgb.prepare();
        ImageOf<PixelFloat>& yarp_float_image = yarp_output_depth.prepare();
        yarp_rgb_image = fromCvMat<PixelRgb>(output_rgb);
        yarp_float_image = fromCvMat<PixelFloat>(output_depth);

        /* Send images. */
        yarp_output_rgb.write();
        yarp_output_depth.write();
    }

    return EXIT_SUCCESS;
}

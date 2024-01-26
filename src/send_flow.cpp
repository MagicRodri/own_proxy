#include <opencv2/opencv.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

#include "flow_px4.hpp"
#include "flow_opencv.hpp"

using namespace mavsdk;
using std::chrono::seconds;

static void subscribe_armed(Telemetry &telemetry);
static void send_optical_flow(MavlinkPassthrough &mavlink_passthrough, float flow_x, float flow_y, int time_us, int quality);

int main(int argc, char const *argv[])
{

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection("udp://0.0.0.0:14551");

    if (connection_result != ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(-1);
    if (!system)
    {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto mavlink_passthrough = MavlinkPassthrough{system.value()};

    // initialize the camera
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cout << "Error opening video stream or file\n";
        return -1;
    }

    auto optflow = OpticalFlowPX4(1596.37f, 1599.17f, -1, 640, 480);
    // auto optflow = OpticalFlowOpenCV(1596.37f, 1599.17f, -1, 640, 480);
    // optflow.setCameraMatrix(1596.37f, 1599.17f, 292.949f, 218.407f);
    // optflow.setCameraDistortion(-0.3707f, -1.836f, 38.703f, 0.000479f, 0.0019187f);
    cv::Mat frame;
    // cv::namedWindow("flow", cv::WINDOW_AUTOSIZE);

    int dt_us, time_us, quality, scale{1};
    float flow_x, flow_y;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;
        double millis = cap.get(cv::CAP_PROP_POS_MSEC);
        time_us = static_cast<int>(millis * 1000);
        quality = optflow.calcFlow(frame.data, time_us, dt_us, flow_x, flow_y);
        std::cout << "flow_x: " << flow_x << "\n"
                  << "flow_y: " << flow_y << "\n"
                  << "dt_us: " << dt_us << "\n"
                  << "time_us: " << time_us << "\n";
        std::cout << "quality: " << quality << "\n";
        std::cout << "\n";

        // cv::imshow("flow", frame);
        // if (cv::waitKey(10) == 27)
        //     break;
        send_optical_flow(mavlink_passthrough, flow_x * scale, flow_y * scale, time_us, quality);
        // send_imu_status(mavlink_passthrough);
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }

    return 0;
}

void send_optical_flow(MavlinkPassthrough &mavlink_passthrough, float flow_x, float flow_y, int time_us, int quality)
{
    mavlink_passthrough.queue_message([&](MavlinkAddress mavlink_address, uint8_t channel)
                                      {
            mavlink_message_t message;
            mavlink_optical_flow_t optical_flow;
            mavlink_msg_optical_flow_pack_chan(
                mavlink_address.system_id,
                mavlink_address.component_id,
                channel,
                &message,
                time_us,
                0,
                flow_x,
                flow_y,
                flow_x,
                flow_y,
                quality,
                0.0f,
                flow_x,
                flow_y
                );
            
            return message; });
}
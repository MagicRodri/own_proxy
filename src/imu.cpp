#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <future>
#include <memory>
#include <thread>
#include <string>

using namespace mavsdk;
using std::chrono::seconds;

static void subscribe_imu(Telemetry &telemetry);
static void subscribe_highres_imu(MavlinkPassthrough &mavlink_passthrough);
static void subscribe_armed(Telemetry &telemetry);
static void set_imu_rate(Telemetry &telemetry, float rate_hz);
static void set_highres_imu_rate(MavlinkPassthrough &mavlink_passthrough, float rate);

int main(int argc, char const *argv[])
{

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    ConnectionResult connection_result = mavsdk.add_any_connection("udp://0.0.0.0:14551");

    if (connection_result != ConnectionResult::Success)
    {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(-1.);
    if (!system)
    {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugins.
    auto telemetry = Telemetry{system.value()};
    auto mavlink_passthrough = MavlinkPassthrough{system.value()};
    set_imu_rate(telemetry, 1.0);
    set_highres_imu_rate(mavlink_passthrough, 200.0);
    // subscribe_imu(telemetry);
    subscribe_highres_imu(mavlink_passthrough);
    // subscribe_armed(telemetry);

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return 0;
}

void subscribe_imu(Telemetry &telemetry)
{

    telemetry.subscribe_raw_imu(
        [](Telemetry::Imu imu)
        {
            auto const [ax, ay, az] = imu.acceleration_frd;
            static unsigned int last_us = 0;
            auto dt = imu.timestamp_us - last_us;
            if (last_us != 0)
            {
                std::cout << "dt: " << dt << " us\n"
                          << "Rate: " << 1e6 / dt << "Hz \n";
            }
            last_us = imu.timestamp_us;
            std::cout << "IMU: \n"
                      << "timestamp: " << imu.timestamp_us << "\n"
                      << "(ax,ay,az): (" << ax << "," << ay << "," << az * 9.81 / 1000 << ")\n";
        });
}

void subscribe_highres_imu(MavlinkPassthrough &mavlink_passthrough)
{
    mavlink_passthrough.subscribe_message(
        MAVLINK_MSG_ID_HIGHRES_IMU,
        [](const mavlink_message_t &message)
        {
            static mavlink_highres_imu_t highres_imu;
            mavlink_msg_highres_imu_decode(&message, &highres_imu);
            static unsigned int last_us = 0;
            auto dt = highres_imu.time_usec - last_us;
            if (last_us != 0 && dt > 0)
            {
                std::cout << "dt: " << dt << " us\n"
                          << "Rate: " << 1e6 / dt << "Hz \n"
                          << "Highres IMU: \n"
                          << "timestamp: " << highres_imu.time_usec << "\n"
                          << "(ax,ay,az): (" << highres_imu.xacc << "," << highres_imu.yacc << "," << highres_imu.zacc << ")\n\n";
            }
            last_us = highres_imu.time_usec;
        });
}

void subscribe_armed(Telemetry &telemetry)
{
    telemetry.subscribe_armed(
        [](bool is_armed)
        { std::cout << (is_armed ? "armed" : "disarmed") << '\n'; });
}

void set_imu_rate(Telemetry &telemetry, float rate_hz = 10.0)
{
    telemetry.set_rate_raw_imu_async(rate_hz, [rate_hz](Telemetry::Result result)
                                     {
        if (result != Telemetry::Result::Success)
        {
            std::cerr << "Setting rate failed: " << result << '\n';
        }
        else
        {
            std::cout << "Rate set to "<<rate_hz<<" Hz\n";
        } });
}

static void set_highres_imu_rate(MavlinkPassthrough &mavlink_passthrough, float rate_hz = 10.0)
{
    mavlink_passthrough.queue_message([&](MavlinkAddress mavlink_address, auto channel)
                                      {
                                          mavlink_message_t message;
                                          mavlink_msg_command_long_pack_chan(
                                              mavlink_address.system_id,
                                              mavlink_address.component_id,
                                              channel,
                                              &message,
                                              0,                            // target_system
                                              0,                            // target_component
                                              MAV_CMD_SET_MESSAGE_INTERVAL, // command
                                              0,                            // confirmation
                                              MAVLINK_MSG_ID_HIGHRES_IMU,   // param1
                                              static_cast<int>(1e6/rate_hz),                      // param2
                                              0,                            // param3
                                              0,                            // param4
                                              0,                            // param5
                                              0,                            // param6
                                              0                             // param7
                                          );
                                          return message; });
}
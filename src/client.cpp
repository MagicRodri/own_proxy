#include <iostream>
#include <thread>
#include <chrono>
#include <string>

#include <SFML/Network.hpp>

int main(int argc, char const *argv[])
{

    sf::UdpSocket socket;

    // UDP socket:
    sf::IpAddress recipient = "127.0.0.1";
    unsigned short port = 54000;

    std::string data{};
    while (true)
    {
        sf::Packet packet;
        std::getline(std::cin, data);
        packet << data;
        if (socket.send(packet, recipient, port) != sf::Socket::Done)
        {
            // error...
        }
    }

    return EXIT_SUCCESS;
}
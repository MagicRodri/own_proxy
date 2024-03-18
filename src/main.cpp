#include <iostream>
#include <thread>
#include <chrono>
#include <string>

#include <SFML/Network.hpp>

int main(int argc, char const *argv[])
{
    sf::UdpSocket socket;
    socket.setBlocking(false);
    if (socket.bind(54000) != sf::Socket::Done)
    {
        // error...
    }

    std::size_t received;

    // UDP socket:
    sf::IpAddress sender;
    unsigned short port;
    sf::Packet packet;
    while (true)
    {
        if (socket.receive(packet, sender, port) != sf::Socket::Done)
        {
            std::cerr << "Error receiving data ...\n";
        }
        else
        {

            if (std::string data{}; packet >> data)
            {
                // ok
                std::cout << "Received: '" << data << "' bytes from " << sender << " on port " << port << std::endl;
            }
            else
            {
                // error, failed to read 'x' from the packet
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    return EXIT_SUCCESS;
}
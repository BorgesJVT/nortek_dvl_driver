#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main() {
    boost::asio::io_context io_context;
    tcp::socket socket(io_context);

    try {
        socket.connect(tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 12345));

        std::string message = "Hello, Server!";
        boost::asio::write(socket, boost::asio::buffer(message));

        socket.shutdown(tcp::socket::shutdown_both);
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}

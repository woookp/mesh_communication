#include "session.h"
#include <iostream>

session::session(boost::asio::ip::tcp::socket socket)
    : socket_(std::move(socket)) {
}

void session::start() {
    socket_.async_read_some(boost::asio::buffer(data_, max_length),
        [self = shared_from_this()](const boost::system::error_code& error, size_t bytes_transferred) {
            self->read_handler(error, bytes_transferred);
        });
}

void session::read_handler(const boost::system::error_code& error, size_t bytes_transferred) {
    if (!error) {
        std::string message(data_, bytes_transferred);
        std::cout << "Received message from " << socket_.remote_endpoint().address().to_string() << ": " << message << std::endl;

        start(); // Continue reading
    } else {
        std::cerr << "Error: " << error.message() << std::endl;
    }
}

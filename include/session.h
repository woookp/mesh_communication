#ifndef SESSION_H
#define SESSION_H

#include <boost/asio.hpp>
#include <memory>

class session : public std::enable_shared_from_this<session> {
public:
    session(boost::asio::ip::tcp::socket socket);
    void start();

private:
    void read_handler(const boost::system::error_code& error, size_t bytes_transferred);
    boost::asio::ip::tcp::socket socket_;
    enum { max_length = 1024 };
    char data_[max_length];
};

#endif // SESSION_H
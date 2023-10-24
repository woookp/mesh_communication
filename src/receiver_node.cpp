// 文件名：mesh_communication/src/receiver_node.cpp
#include <ros/ros.h>
#include <mesh_communication/StringMessage.h>
#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using namespace std;

void dataCallback(const mesh_communication::StringMessage::ConstPtr &msg) {
  ROS_INFO("Received data: %s", msg->data.c_str());
}

void handle_read(const boost::system::error_code& ec, std::size_t bytes_transferred,
                 boost::asio::streambuf& buf, ros::Publisher& pub, tcp::socket& socket) {
    if (!ec) {
        // 处理读取到的数据
        std::string data = boost::asio::buffer_cast<const char*>(buf.data());
        cout << data << endl;
        buf.consume(bytes_transferred);

        // 将接收到的数据转换为 ROS 消息并发布
        mesh_communication::StringMessage msg;
        msg.data = data;
        pub.publish(msg);

        // 再次启动异步读取操作
        boost::asio::async_read_until(socket, buf, '!',
                                    std::bind(handle_read, std::placeholders::_1,
                                              std::placeholders::_2, std::ref(buf), std::ref(pub), std::ref(socket)));
    } else {
        cerr << "Read error: " << ec.message() << endl;
        socket.close();  // 关闭套接字，终止连接
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "status_node");
  ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("mesh_data", 1, dataCallback);
  ros::Publisher pub = nh.advertise<mesh_communication::StringMessage>("net2/status", 1);
  uint16_t port = 12345;
  ros::Rate loop_rate(10);

  // 初始化 Boost.Asio
  boost::asio::io_service io_service;
  tcp::acceptor acceptor(io_service, tcp::endpoint(boost::asio::ip::make_address("0.0.0.0"), port));

  try {
      tcp::socket socket(io_service);
      acceptor.accept(socket);
      boost::system::error_code ec;
      boost::asio::streambuf buf;
      buf.prepare(1024);
      boost::asio::async_read_until(socket, buf, '!',
                                  std::bind(handle_read, std::placeholders::_1,
                                            std::placeholders::_2, std::ref(buf), std::ref(pub), std::ref(socket)));
      while (ros::master::check() && ros::ok() && socket.is_open()) {
          // 启动异步读取操作
          io_service.poll();  // 运行一次 I/O 服务
          // 等待一段时间，以控制读取频率
          //loop_rate.sleep();
      }
      
  } catch (exception &e) {
    cerr << "Exception: " << e.what() << endl;
  }

  cout << "End of program" << endl;
  return 0;
}

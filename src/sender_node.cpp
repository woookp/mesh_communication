#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/CompressedImage.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

using boost::asio::ip::tcp;

class VideoSender {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber odom_sub;
    ros::Subscriber compressed_image_sub;
    boost::asio::io_service io_service;
    tcp::socket socket;
    tcp::resolver resolver;
    boost::system::error_code ec;
    ros::Subscriber pc_sub;
    bool is_blocked = false;
    bool reduce_frame_rate = false;

public:
    VideoSender() : socket(io_service), resolver(io_service){
        std::string target_ip = "192.168.142.145";
        uint16_t target_port = 12345;
        // std::string target_ip = argv[1]; // 第一个参数作为IP地址
        // uint16_t target_port = static_cast<uint16_t>(std::atoi(argv[2])); // 第二个参数作为端口号
        // 解析目标 IP 地址和端口号
        boost::asio::ip::tcp::resolver::query query(target_ip, std::to_string(target_port));
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::connect(socket, endpoint_iterator, ec);
        while (ec && ros::master::check() && ros::ok()) {
            std::cerr << "Error connecting to target IP: " << ec.message() << ". Still trying." << std::endl;
            boost::asio::connect(socket, endpoint_iterator, ec);
            ros::Duration(1).sleep();
        }
        // 订阅图像topic
        odom_sub = nh.subscribe("/Odometry", 100, &VideoSender::odometryCallback, this);
        sub = nh.subscribe("/usb_cam/image_raw", 100, &VideoSender::imageCallback, this);
        pc_sub = nh.subscribe("/merged", 100, &VideoSender::pointCloudCallback, this);
        compressed_image_sub = nh.subscribe("/camera/color/image_raw/compressed", 100, &VideoSender::compressedImageCallback, this);
    }

    void appendDataToBuffer(std::vector<uchar>& buffer, const void* data, size_t size) {
        const uchar* byteData = reinterpret_cast<const uchar*>(data);
        buffer.insert(buffer.end(), byteData, byteData + size);
    }

    void write_with_timeout(tcp::socket &socket, const std::vector<uchar> &data, boost::system::error_code &ec) {
        auto start = std::chrono::high_resolution_clock::now();
        boost::asio::write(socket, boost::asio::buffer(data.data(), data.size()), ec);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;

        if (elapsed.count() > 0.03) {
            is_blocked = true;
            std::cout << "Channel is blocked." << std::endl;
            if (!reduce_frame_rate) {
                reduce_frame_rate = true;
            }
        } else {
            // is_blocked = false;
        }
    }


    void reconnect() {
        socket.close(); // 关闭旧套接字
        socket = tcp::socket(io_service); // 创建新套接字
        std::string target_ip = "192.168.142.145";
        uint16_t target_port = 12345;
        boost::asio::ip::tcp::resolver::query query(target_ip, std::to_string(target_port));
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::connect(socket, endpoint_iterator, ec);
        while (ec) {
            std::cerr << "Error reconnecting: " << ec.message() << ". Retrying..." << std::endl;
            boost::asio::connect(socket, endpoint_iterator, ec);
            ros::Duration(1).sleep();
        }
        std::cout << "Reconnected successfully." << std::endl;
    }

    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        // 处理压缩图像并发送
        std::vector<uchar> sendBuffer;
        uint8_t dataType = 0x04; // CompressedImage data
        sendBuffer.push_back(dataType);

        uint32_t dataSize = msg->data.size();
        uint32_t formatSize = msg->format.size();

        // 添加dataSize
        appendDataToBuffer(sendBuffer, &dataSize, sizeof(dataSize));

        // 添加formatSize
        appendDataToBuffer(sendBuffer, &formatSize, sizeof(formatSize));

        for(uint8_t i : msg->data){
            appendDataToBuffer(sendBuffer, &i, sizeof(uint8_t));
        }

        // 添加msg->format内容
        for (auto i : msg->format){
            appendDataToBuffer(sendBuffer, &i, sizeof(char));
        }
        // sendBuffer.insert(sendBuffer, msg->format.begin(), msg->format.end());

        // 添加msg->data内容

        // sendBuffer.insert(sendBuffer.end(), msg->data.begin(), msg->data.end());
        // if(sendBuffer.size() == dataSize + formatSize + 2*sizeof(uint32_t)){
        //     std::cout << "size == dataSize + formatSize" << std::endl;
        // }else{
        // std::cout << "senderbuffer.size:" << sendBuffer.size() << std::endl;
        // std::cout << "datasize:" << dataSize << std::endl;
        // std::cout << "formatsize:" << formatSize << std::endl;
        // std::cout << "sizeof(uint32_t)" << sizeof(u_int32_t) << std::endl;
        // }
        boost::system::error_code ec;
        write_with_timeout(socket, sendBuffer, ec);
        if(ec) {
        // 处理错误
            std::cerr << "Error while writing: " << ec.message() << std::endl;
            reconnect();
        }
        // 使用Boost.Asio异步发送 sendBuffer
        // boost::asio::async_write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()),
        //     boost::bind(&VideoSender::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::vector<uchar> sendBuffer;
        uint8_t dataType = 0x03; // PointCloud2 data
        sendBuffer.push_back(dataType);

        // 将data信息添加到sendBuffer中
        uint32_t dataSize = msg->data.size();
        // std::cout << "pointCloud datasize:" << dataSize << std::endl;

        appendDataToBuffer(sendBuffer, &dataSize, sizeof(dataSize));
        // appendDataToBuffer(sendBuffer, msg->header.frame_id, frame_id_size);



        // 将header信息添加到sendBuffer中
        uint32_t seq = msg->header.seq;
        appendDataToBuffer(sendBuffer, &seq, sizeof(seq));
        ros::Time stamp = msg->header.stamp;
        uint32_t sec = stamp.sec;
        uint32_t nsec = stamp.nsec;
        appendDataToBuffer(sendBuffer, &sec, sizeof(sec));
        appendDataToBuffer(sendBuffer, &nsec, sizeof(nsec));
        // 将height, width, is_dense, is_bigendian, point_step, row_step添加到sendBuffer中
        appendDataToBuffer(sendBuffer, &msg->height, sizeof(msg->height));
        appendDataToBuffer(sendBuffer, &msg->width, sizeof(msg->width));

        bool is_bigendian = msg->is_bigendian;
        uint32_t point_step = msg->point_step;
        uint32_t row_step = msg->row_step;

        appendDataToBuffer(sendBuffer, &is_bigendian, sizeof(is_bigendian));
        appendDataToBuffer(sendBuffer, &point_step, sizeof(point_step));
        appendDataToBuffer(sendBuffer, &row_step, sizeof(row_step));

        bool is_dense = msg->is_dense;
        appendDataToBuffer(sendBuffer, &is_dense, sizeof(is_dense));

        for (const auto& field: msg->fields) {
            uint32_t offset = field.offset;
            uint8_t datatype = field.datatype;
            uint32_t count = field.count;
            appendDataToBuffer(sendBuffer, &offset, sizeof(field.offset));
            appendDataToBuffer(sendBuffer, &datatype, sizeof(field.datatype));
            appendDataToBuffer(sendBuffer, &count, sizeof(field.count));
        }

        // for(uint8_t sub_data : msg->data){

        // }
        // appendDataToBuffer(sendBuffer, msg->data.data(), dataSize);
        // sendBuffer.insert(sendBuffer.end(), msg->data.begin(), msg->data.end());
        for(uint8_t i : msg->data){
            appendDataToBuffer(sendBuffer, &i, sizeof(uint8_t));
            dataSize--;
        }
        // std::cout << "datasize--:" << dataSize << std::endl;
        // 使用Boost.Asio异步发送 sendBuffer
        boost::system::error_code ec;
        write_with_timeout(socket, sendBuffer, ec);
        if(ec) {
        // 处理错误
            std::cerr << "Error while writing: " << ec.message() << std::endl;
            reconnect();
        }
        // std::cout << "data insert down" << std::endl;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::vector<uchar> sendBuffer;
        // 添加数据类型头部
        uint8_t dataType = 0x02; // Odometry data
        sendBuffer.push_back(dataType);
        // 添加时间戳
        uint32_t sec = msg->header.stamp.sec;
        uint32_t nsec = msg->header.stamp.nsec;
        appendDataToBuffer(sendBuffer, &sec, sizeof(sec));
        appendDataToBuffer(sendBuffer, &nsec, sizeof(nsec));
        // 1. 编码位置数据
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        appendDataToBuffer(sendBuffer, &x, sizeof(x));
        appendDataToBuffer(sendBuffer, &y, sizeof(y));
        appendDataToBuffer(sendBuffer, &z, sizeof(z));

        // 2. 编码方向（四元数）数据
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        appendDataToBuffer(sendBuffer, &qx, sizeof(qx));
        appendDataToBuffer(sendBuffer, &qy, sizeof(qy));
        appendDataToBuffer(sendBuffer, &qz, sizeof(qz));
        appendDataToBuffer(sendBuffer, &qw, sizeof(qw));

        // 3. 编码线速度
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        double vz = msg->twist.twist.linear.z;
        appendDataToBuffer(sendBuffer, &vx, sizeof(vx));
        appendDataToBuffer(sendBuffer, &vy, sizeof(vy));
        appendDataToBuffer(sendBuffer, &vz, sizeof(vz));

        // 4. 编码角速度
        double wx = msg->twist.twist.angular.x;
        double wy = msg->twist.twist.angular.y;
        double wz = msg->twist.twist.angular.z;
        appendDataToBuffer(sendBuffer, &wx, sizeof(wx));
        appendDataToBuffer(sendBuffer, &wy, sizeof(wy));
        appendDataToBuffer(sendBuffer, &wz, sizeof(wz));

        // // 5. 使用Boost.Asio异步发送 sendBuffer
        // boost::asio::async_write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()),
        //     boost::bind(&VideoSender::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        //同步发送
        boost::system::error_code ec;
        write_with_timeout(socket, sendBuffer, ec);
        if(ec) {
            std::cerr << "Error while writing: " << ec.message() << std::endl;
            reconnect();
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // 处理图像并发送
        try {
            // 使用cv_bridge将ROS图像消息转换为OpenCV图像
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::vector<uchar> sendBuffer;

           cv::Mat resizedFrame;
           double scale = 1; // 缩小到原来的50%
           cv::resize(frame, resizedFrame, cv::Size(), scale, scale, cv::INTER_LINEAR);

            // 将帧转换为字节数组 (这只是一个简化的示例)
            std::vector<uchar> buf;
            cv::imencode(".jpg", resizedFrame, buf); 

            uint8_t dataType = 0x01; // Image data
            sendBuffer.push_back(dataType);

            // 创建一个大的缓冲区来包含 dataSize 和 buf 的数据
            uint32_t dataSize = buf.size();   

            // 首先将 dataSize 转换为字节数组并添加到 sendBuffer
            for (int i = 0; i < sizeof(dataSize); ++i) {
                sendBuffer.push_back(((uchar*)&dataSize)[i]);
            }

            // 然后将 buf 的内容添加到 sendBuffer
            sendBuffer.insert(sendBuffer.end(), buf.begin(), buf.end());

            boost::system::error_code ec;
            write_with_timeout(socket, sendBuffer, ec);
            // 使用Boost.Asio异步发送 sendBuffer
            // boost::asio::async_write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()),
            //     boost::bind(&VideoSender::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            // 使用Boost.Asio同步发送 sendBuffer
        } catch (cv_bridge::Exception& e) {
            std::cerr << "Could not convert from '" << msg->encoding << "' to 'bgr8'." << std::endl;
        }
    }


    void sendCommandData(uint32_t data) {
        std::vector<uchar> sendBuffer;
        uint8_t dataType = 0x05; // 新的数据类型标识符，例如 0x05
        sendBuffer.push_back(dataType);

        // 将数据转换为字节数组并添加到缓冲区
        appendDataToBuffer(sendBuffer, &data, sizeof(data));

        // 发送数据
        boost::system::error_code ec;
        boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
        if(ec) {
            std::cerr << "Error while writing: " << ec.message() << std::endl;
            reconnect();
        }
    }

    void run() {  
        while (ros::ok()) {
            ros::Rate loop_rate(10);
            if(is_blocked){
                loop_rate = ros::Rate(1); // 设置为1Hz
            }else{
                loop_rate = ros::Rate(10); // 设置为10Hz
            }
            io_service.reset();
            // 获取参数服务器上的指令数据
            int commandDataInt;
            if (nh.getParam("/command_param", commandDataInt)) {
                uint32_t commandData = static_cast<uint32_t>(commandDataInt);
                sendCommandData(commandData);
            } else {
                ROS_WARN("Failed to get command data,name: /command_param");
            }
            ros::spinOnce();
            io_service.run_one();
            loop_rate.sleep();
        }
        socket.close();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_sender");
    ros::NodeHandle nh;
    VideoSender sender;
    sender.run();
    return 0;
}

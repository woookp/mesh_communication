#include <ros/ros.h>
#include <sensor_msgs/Image.h>
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
    boost::asio::io_service io_service;
    tcp::socket socket;
    tcp::resolver resolver;
    boost::system::error_code ec;
    ros::Subscriber pc_sub;

public:
    VideoSender() : socket(io_service), resolver(io_service){
        std::string target_ip = "100.78.35.142";
        uint16_t target_port = 12345;
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
        odom_sub = nh.subscribe("/Odometry", 1, &VideoSender::odometryCallback, this);
        sub = nh.subscribe("/camera/color/image_raw", 1, &VideoSender::imageCallback, this);
        pc_sub = nh.subscribe("/merged", 1, &VideoSender::pointCloudCallback, this);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        std::vector<uchar> sendBuffer;
        uint8_t dataType = 0x03; // PointCloud2 data
        sendBuffer.push_back(dataType);

        // 将data信息添加到sendBuffer中
        uint32_t dataSize = msg->data.size();
        std::cout << "pointCloud datasize:" << dataSize << std::endl;

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
        std::cout << "datasize--:" << dataSize << std::endl;
        // 使用Boost.Asio异步发送 sendBuffer
        boost::system::error_code ec;
        boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
        if(ec) {
        // 处理错误
            std::cerr << "Error while writing: " << ec.message() << std::endl;
        }
        std::cout << "data insert down" << std::endl;
    }

    void appendDataToBuffer(std::vector<uchar>& buffer, const void* data, size_t size) {
        const uchar* byteData = reinterpret_cast<const uchar*>(data);
        buffer.insert(buffer.end(), byteData, byteData + size);
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::vector<uchar> sendBuffer;
        // 添加数据类型头部
        uint8_t dataType = 0x02; // Odometry data
        sendBuffer.push_back(dataType);

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

        // 5. 使用Boost.Asio异步发送 sendBuffer
        boost::asio::async_write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()),
            boost::bind(&VideoSender::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // 处理图像并发送
        try {
            // 使用cv_bridge将ROS图像消息转换为OpenCV图像
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::vector<uchar> sendBuffer;

           cv::Mat resizedFrame;
           double scale = 0.5; // 缩小到原来的50%
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

            // 使用Boost.Asio异步发送 sendBuffer
            boost::asio::async_write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()),
                boost::bind(&VideoSender::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            // 使用Boost.Asio同步发送 sendBuffer


        } catch (cv_bridge::Exception& e) {
            std::cerr << "Could not convert from '" << msg->encoding << "' to 'bgr8'." << std::endl;
        }
    }

    void handle_write(const boost::system::error_code& ec, std::size_t bytes_transferred) {
        if (ec) {
            std::cerr << "Error sending data: " << ec.message() << std::endl;
        }
    }

    void run() {
        ros::Rate loop_rate(1);
        while (ros::ok()) {
            io_service.reset();
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

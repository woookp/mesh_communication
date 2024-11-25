#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/CompressedImage.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "visualization_msgs/Marker.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

using boost::asio::ip::tcp;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
std::string target_ip_yaml;
int target_port_yaml;
std::string laser_node_name;
std::string odom_node_name;
uint16_t target_port;
std::string target_ip;
class VideoSender
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub = nullptr;
    ros::Subscriber compressed_image_sub;
    boost::asio::io_service io_service;
    tcp::socket socket;
    tcp::resolver resolver;
    boost::system::error_code ec;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *pc_sub = nullptr;
    ros::Subscriber marker_sub;
    boost::asio::deadline_timer deadline;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    friend void reconnectSocketThread(VideoSender *sender);
    // 队列：图像和Odometry数据
    std::queue<sensor_msgs::PointCloud2::ConstPtr> cloudQueue;
    std::queue<std::vector<uchar>> imageQueue;

    std::queue<nav_msgs::Odometry::ConstPtr> odomQueue;
    size_t cloudQueueSize = 10000; // 点云队列最大容量 test
    size_t odomQueueSize = 10000;  // Odometry队列最大容量
    size_t iamgQueueSize = 10000;  // Odometry队列最大容量

    std::mutex imageQueueMutex; // 图像队列互斥锁
    std::mutex cloudQueueMutex; // 点云队列互斥锁
    std::mutex odomQueueMutex;  // Odometry队列互斥锁
    std::mutex Mutex;           // sender互斥锁
    std::mutex socket_mutex;    // socket互斥锁

public:
    VideoSender() : socket(io_service), resolver(io_service), deadline(io_service)
    {

        // 从yaml文件获得IP参数, 可以使用roslaunch获取

        // std::string target_ip = argv[1]; // 第一个参数作为IP地址
        // uint16_t target_port = static_cast<uint16_t>(std::atoi(argv[2])); // 第二个参数作为端口号
        // std::string target_ip = "192.168.1.18";
        // uint16_t target_port = 12345;
        // 解析目标 IP 地址和端口号
        boost::asio::ip::tcp::resolver::query query(target_ip, std::to_string(target_port));
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::connect(socket, endpoint_iterator, ec);

        while (ec && ros::master::check() && ros::ok())
        {
            std::cerr << "Error connecting to target IP: " << ec.message() << ". Still trying." << std::endl;
            boost::asio::connect(socket, endpoint_iterator, ec);
            ros::Duration(1).sleep();
        }
        ROS_INFO("connection sucess");

        deadline.expires_from_now(boost::posix_time::seconds(20));
        deadline.async_wait(boost::bind(&VideoSender::handle_timeout, this, boost::asio::placeholders::error));

        // 订阅图像topic
        int node_id = 0;
        nh.param<int>("node_id", node_id, 1);

        // std::string node_name_map_c_submap = "/sub_submap_" + std::to_string(node_id);
        // std::string node_name_map_c_odom = "/sub_odom_" + std::to_string(node_id);
        std::string node_name_map_c_submap = "/sub_submap_1";
        std::string node_name_map_c_odom = "/sub_odom_1";
        nh.param<std::string>("laser_name", node_name_map_c_submap, "/cloud_registered_body");
        nh.param<std::string>("Odometry", node_name_map_c_odom, "/Odometry");

        auto *odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(nh, node_name_map_c_odom, 1000);
        auto *pc_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, node_name_map_c_submap, 1000);
        compressed_image_sub = nh.subscribe("/camera/color/image_raw/compressed", 1, &VideoSender::compressedImageCallback, this);
        marker_sub = nh.subscribe("/visualization_marker", 1, &VideoSender::markerCallback, this);
        message_filters::Synchronizer<MySyncPolicy> *sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(1000), *odom_sub, *pc_sub);
        sync->registerCallback(boost::bind(&VideoSender::syncOdomAndCloudCallback, this, _1, _2));
    }

    void syncOdomAndCloudCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        // 加锁保护队列
        {
            std::lock_guard<std::mutex> lock(cloudQueueMutex);
            if (cloudQueue.size() >= cloudQueueSize)
            {
                cloudQueue.pop(); // 如果队列满了，丢弃最旧的消息
            }
            cloudQueue.push(cloud_msg);
        }
        {

            if (odomQueue.size() >= odomQueueSize)
            {
                odomQueue.pop(); // 如果队列满了，丢弃最旧的消息
            }
            odomQueue.push(odom_msg);
        }
        // 尝试发送数据
        sendSyncOdomAndCloudData();

        // add to package.xml
        // add to CMakeLists.txt
        return;
    }

    // 发送队列中的 Odometry 数据
    void sendSyncOdomAndCloudData()
    {
        while (true)
        {
            nav_msgs::Odometry::ConstPtr odom_msg;
            sensor_msgs::PointCloud2::ConstPtr cloud_msg;

            // 从队列中提取消息
            {
                std::lock_guard<std::mutex> lock(Mutex);
                if (odomQueue.empty() || cloudQueue.empty())
                    break; // 队列为空，结束发送

                odom_msg = odomQueue.front();
                cloud_msg = cloudQueue.front();
            }
            // 将消息编码到缓冲区
            std::vector<uchar> sendBuffer;
            appendSyncOdomAndClouToBuffer(odom_msg, cloud_msg, sendBuffer);

            // 使用 Boost.Asio 发送数据
            boost::system::error_code ec;
            boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
            std::lock_guard<std::mutex> lock(socket_mutex);

            if (ec)
            {
                std::cerr << "Error while writing sync data: " << ec.message() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 调整延迟时间，例如1秒
                // print queue size
                std::lock_guard<std::mutex> lock(Mutex);
                std::cout << "sync size:" << odomQueue.size() << std::endl;
                socket.close();
                return; // 停止发送，等待重连
            }
            else
            {
                std::lock_guard<std::mutex> lock(Mutex);
                odomQueue.pop();
                cloudQueue.pop();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
        }
    }

    /**
     * @brief Appends synchronized odometry and point cloud data to the buffer.
     *
     * This function takes in odometry and point cloud messages, and appends them
     * to the provided buffer in a specific format. The data type identifier for
     * synchronized odometry and point cloud is added to the buffer before the
     * actual data.
     *
     * @param odom_msg The odometry message to be appended.
     * @param pc_msg The point cloud message to be appended.
     * @param sendBuffer The buffer to which the data will be appended.
     */
    void appendSyncOdomAndClouToBuffer(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::PointCloud2::ConstPtr &pc_msg, std::vector<uchar> &sendBuffer)
    {

        uint8_t dataType = 0x07; // sync odom and point cloud
        sendBuffer.push_back(dataType);
        appendOdometryToBuffer(odom_msg, sendBuffer);
        appendPointCloudToBuffer(pc_msg, sendBuffer);
        // print buffer size
        std::cout << "sendBuffer size:" << sendBuffer.size() << std::endl;
    }

    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
    {
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

        for (uint8_t i : msg->data)
        {
            appendDataToBuffer(sendBuffer, &i, sizeof(uint8_t));
        }

        // 添加msg->format内容
        for (auto i : msg->format)
        {
            appendDataToBuffer(sendBuffer, &i, sizeof(char));
        }
        // sendBuffer.insert(sendBuffer, msg->format.begin(), msg->format.end());

        // 添加msg->data内容

        // sendBuffer.insert(sendBuffer.end(), msg->data.begin(), msg->data.end());
        // if(sendBuffer.size() == dataSize + formatSize + 2*sizeof(uint32_t)){
        //     std::cout << "size == dataSize + formatSize" << std::endl;
        // }else{
        std::cout << "senderbuffer.size:" << sendBuffer.size() << std::endl;
        std::cout << "datasize:" << dataSize << std::endl;
        std::cout << "formatsize:" << formatSize << std::endl;
        std::cout << "sizeof(uint32_t)" << sizeof(u_int32_t) << std::endl;
        // }

        boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
        std::lock_guard<std::mutex> lock(socket_mutex);
        if (ec)
        {
            // 处理错误
            std::cerr << "Error while writing: " << ec.message() << std::endl;
            socket.close();
            return;
            // reconnect();
        }
        // 使用Boost.Asio异步发送 sendBuffer
        // boost::asio::async_write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()),
        //     boost::bind(&VideoSender::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }

    void appendDataToBuffer(std::vector<uchar> &buffer, const void *data, size_t size)
    {
        const uchar *byteData = reinterpret_cast<const uchar *>(data);
        buffer.insert(buffer.end(), byteData, byteData + size);
    }

    // 添加 pc2 消息到缓冲区的工具函数
    void appendPointCloudToBuffer(const sensor_msgs::PointCloud2::ConstPtr &msg, std::vector<uchar> &sendBuffer)
    {

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
        // print sec and nsec
        std::cout << "sec:" << sec << std::endl;
        std::cout << "nsec:" << nsec << std::endl;
        // 将height, width, is_dense, is_bigendian, point_step, row_step添加到sendBuffer中
        appendDataToBuffer(sendBuffer, &msg->height, sizeof(msg->height));
        appendDataToBuffer(sendBuffer, &msg->width, sizeof(msg->width));
        std::cout << "height:" << msg->height << std::endl;
        std::cout << "width:" << msg->width << std::endl;
        uint8_t is_bigendian = msg->is_bigendian;
        uint32_t point_step = msg->point_step;
        uint32_t row_step = msg->row_step;
        std::cout << "is_bigendian:" << is_bigendian << std::endl;
        std::cout << "point_step:" << point_step << std::endl;
        std::cout << "row_step:" << row_step << std::endl;

        appendDataToBuffer(sendBuffer, &is_bigendian, sizeof(is_bigendian));
        appendDataToBuffer(sendBuffer, &point_step, sizeof(point_step));
        appendDataToBuffer(sendBuffer, &row_step, sizeof(row_step));

        uint8_t is_dense = msg->is_dense;
        appendDataToBuffer(sendBuffer, &is_dense, sizeof(is_dense));
        std::cout << "is_dense:" << is_dense << std::endl;

        // for (const auto &field : msg->fields)
        // {
        //     uint32_t offset = field.offset;
        //     uint8_t datatype = field.datatype;
        //     uint32_t count = field.count;
        //     appendDataToBuffer(sendBuffer, &offset, sizeof(field.offset));
        //     appendDataToBuffer(sendBuffer, &datatype, sizeof(field.datatype));
        //     appendDataToBuffer(sendBuffer, &count, sizeof(field.count));
        // }
        for (const auto &field : msg->fields)
        {
            uint32_t offset = field.offset;
            uint8_t datatype = field.datatype;
            uint32_t count = field.count;

            // 序列化字段名
            // uint32_t name_size = field.name.size();
            // appendDataToBuffer(sendBuffer, &name_size, sizeof(name_size));
            // appendDataToBuffer(sendBuffer, field.name.c_str(), name_size);

            // 序列化其他字段
            appendDataToBuffer(sendBuffer, &offset, sizeof(field.offset));
            appendDataToBuffer(sendBuffer, &datatype, sizeof(field.datatype));
            appendDataToBuffer(sendBuffer, &count, sizeof(field.count));
        }
        // for(uint8_t sub_data : msg->data){

        // }
        // appendDataToBuffer(sendBuffer, msg->data.data(), dataSize);
        // sendBuffer.insert(sendBuffer.end(), msg->data.begin(), msg->data.end());
        // for (uint8_t i : msg->data)
        // {
        //     appendDataToBuffer(sendBuffer, &i, sizeof(uint8_t));
        //     dataSize--;
        // }
        for (uint8_t i : msg->data)
        {
            appendDataToBuffer(sendBuffer, &i, sizeof(uint8_t));
        }
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        // 加锁保护队列
        {
            std::lock_guard<std::mutex> lock(cloudQueueMutex);
            if (cloudQueue.size() >= cloudQueueSize)
            {
                cloudQueue.pop(); // 如果队列满了，丢弃最旧的消息
            }
            cloudQueue.push(msg);
        }

        // 尝试发送数据
        sendPointCloudData();
    }

    // 发送队列中的 Odometry 数据
    void sendPointCloudData()
    {
        while (true)
        {
            sensor_msgs::PointCloud2::ConstPtr msg;

            // 从队列中提取消息
            {
                std::lock_guard<std::mutex> lock(cloudQueueMutex);
                if (cloudQueue.empty())
                    break; // 队列为空，结束发送

                msg = cloudQueue.front();
            }

            // 将消息编码到缓冲区
            uint32_t dataSize = msg->data.size();
            std::vector<uchar> sendBuffer;
            uint8_t dataType = 0x03; // PointCloud2 data
            sendBuffer.push_back(dataType);
            appendPointCloudToBuffer(msg, sendBuffer);
            std::cout << "datasize--:" << dataSize << std::endl;
            // 使用 Boost.Asio 发送数据
            boost::system::error_code ec;
            boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
            std::lock_guard<std::mutex> lock(socket_mutex);

            if (ec)
            {
                std::cerr << "Error while writing pointcloud data: " << ec.message() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 调整延迟时间，例如1秒
                // print queue size
                std::lock_guard<std::mutex> lock(cloudQueueMutex);
                std::cout << "cloudQueue size:" << cloudQueue.size() << std::endl;
                socket.close();
                return; // 停止发送，等待重连
            }
            else
            {
                std::lock_guard<std::mutex> lock(cloudQueueMutex);
                cloudQueue.pop();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            std::cout << "data insert down" << std::endl;
        }
    }

    void markerCallback(const visualization_msgs::Marker::ConstPtr &msg)
    {
        // marker into socket buffer

        std::vector<uchar> sendBuffer;
        uint8_t dataType = 0x06; // Marker data
        sendBuffer.push_back(dataType);
        // 添加时间戳
        uint32_t sec = msg->header.stamp.sec;
        uint32_t nsec = msg->header.stamp.nsec;
        appendDataToBuffer(sendBuffer, &sec, sizeof(sec));
        appendDataToBuffer(sendBuffer, &nsec, sizeof(nsec));
        // 添加id
        uint32_t id = msg->id;
        appendDataToBuffer(sendBuffer, &id, sizeof(id));
        // 添加action
        uint32_t action = msg->action;
        appendDataToBuffer(sendBuffer, &action, sizeof(action));
        uint32_t type = msg->type;
        appendDataToBuffer(sendBuffer, &type, sizeof(type));
        ROS_INFO("marker type:%d,action:%d,id:%d", type, action, id);
        // 添加pose
        double px = msg->pose.position.x;
        double py = msg->pose.position.y;
        double pz = msg->pose.position.z;
        appendDataToBuffer(sendBuffer, &px, sizeof(px));
        appendDataToBuffer(sendBuffer, &py, sizeof(py));
        appendDataToBuffer(sendBuffer, &pz, sizeof(pz));
        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        appendDataToBuffer(sendBuffer, &qx, sizeof(qx));
        appendDataToBuffer(sendBuffer, &qy, sizeof(qy));
        appendDataToBuffer(sendBuffer, &qz, sizeof(qz));
        appendDataToBuffer(sendBuffer, &qw, sizeof(qw));

        // 同步发送
        boost::system::error_code ec;
        boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
        std::lock_guard<std::mutex> lock(socket_mutex);
        if (ec)
        {
            std::cerr << "Error while writing: Marker" << ec.message() << std::endl;
            socket.close();
            return;
            // reconnect();
        }
    }

    // 添加 Odometry 消息到缓冲区的工具函数
    void appendOdometryToBuffer(const nav_msgs::Odometry::ConstPtr &msg, std::vector<uchar> &sendBuffer)
    {

        // 添加时间戳
        uint32_t sec = msg->header.stamp.sec;
        uint32_t nsec = msg->header.stamp.nsec;
        appendDataToBuffer(sendBuffer, &sec, sizeof(sec));
        appendDataToBuffer(sendBuffer, &nsec, sizeof(nsec));

        // 编码位置数据
        appendDataToBuffer(sendBuffer, &msg->pose.pose.position.x, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->pose.pose.position.y, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->pose.pose.position.z, sizeof(double));

        // 编码方向（四元数）数据
        appendDataToBuffer(sendBuffer, &msg->pose.pose.orientation.x, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->pose.pose.orientation.y, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->pose.pose.orientation.z, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->pose.pose.orientation.w, sizeof(double));

        // 编码线速度
        appendDataToBuffer(sendBuffer, &msg->twist.twist.linear.x, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->twist.twist.linear.y, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->twist.twist.linear.z, sizeof(double));

        // 编码角速度
        appendDataToBuffer(sendBuffer, &msg->twist.twist.angular.x, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->twist.twist.angular.y, sizeof(double));
        appendDataToBuffer(sendBuffer, &msg->twist.twist.angular.z, sizeof(double));
    }

    // 回调函数：接收 Odometry 消息并加入队列
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 加锁保护队列
        {
            std::lock_guard<std::mutex> lock(odomQueueMutex);
            if (odomQueue.size() >= odomQueueSize)
            {
                odomQueue.pop(); // 如果队列满了，丢弃最旧的消息
            }
            odomQueue.push(msg);
        }

        // 尝试发送数据
        sendOdometryData();
    }

    // 发送队列中的 Odometry 数据
    void sendOdometryData()
    {
        while (true)
        {
            nav_msgs::Odometry::ConstPtr msg;

            // 从队列中提取消息
            {
                std::lock_guard<std::mutex> lock(odomQueueMutex);
                if (odomQueue.empty())
                    break; // 队列为空，结束发送

                msg = odomQueue.front();
            }

            // 将消息编码到缓冲区
            std::vector<uchar> sendBuffer;
            uint8_t dataType = 0x02; // Odometry data
            sendBuffer.push_back(dataType);
            appendOdometryToBuffer(msg, sendBuffer);

            // 使用 Boost.Asio 发送数据
            boost::system::error_code ec;
            boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
            std::lock_guard<std::mutex> lock(socket_mutex);

            if (ec)
            {
                std::cerr << "Error while writing odometry data: " << ec.message() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1)); // 调整延迟时间，例如1秒
                // print queue size
                std::lock_guard<std::mutex> lock(odomQueueMutex);
                std::cout << "odomQueue size:" << odomQueue.size() << std::endl;
                socket.close();
                return; // 停止发送，等待重连
            }
            else
            {
                std::lock_guard<std::mutex> lock(odomQueueMutex);
                odomQueue.pop();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
        }
    }
    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        // 处理图像并发送
        try
        {
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
            for (int i = 0; i < sizeof(dataSize); ++i)
            {
                sendBuffer.push_back(((uchar *)&dataSize)[i]);
            }

            // 然后将 buf 的内容添加到 sendBuffer
            sendBuffer.insert(sendBuffer.end(), buf.begin(), buf.end());

            // 使用Boost.Asio异步发送 sendBuffer
            boost::asio::async_write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()),
                                     boost::bind(&VideoSender::handle_write, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            // 使用Boost.Asio同步发送 sendBuffer
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "Could not convert from '" << msg->encoding << "' to 'bgr8'." << std::endl;
        }
    }

    void sendImageData()
    {
        std::lock_guard<std::mutex> lock(imageQueueMutex);

        while (!imageQueue.empty())
        {
            std::vector<uchar> sendBuffer = imageQueue.front();
            boost::system::error_code ec;

            // 使用 Boost.Asio 发送数据
            boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
            std::lock_guard<std::mutex> lock(socket_mutex);

            if (ec)
            {
                std::cerr << "Error while writing image data: " << ec.message() << std::endl;
                socket.close();
                return;
                // reconnect();
                return; // 停止处理队列，等待重连
            }
            else
            {
                imageQueue.pop(); // 发送成功，移除图像队列中的数据
            }
        }
    }

    void handle_write(const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
        std::lock_guard<std::mutex> lock(socket_mutex);
        if (ec)
        {
            std::cerr << "Error sending data: " << ec.message() << std::endl;
            std::cerr << "try reconnect " << ec.message() << std::endl;
            socket.close();
            return;
            // reconnect();
        }
    }
    void handle_timeout(const boost::system::error_code &error)
    {
        if (!error)
        {
            // 超时处理代码
            std::cout << "Write operation timed out." << std::endl;
            return;
            // reconnect();
        }
    }

    void sendCommandData(uint32_t data)
    {
        std::vector<uchar> sendBuffer;
        uint8_t dataType = 0x05; // 新的数据类型标识符，例如 0x05
        sendBuffer.push_back(dataType);

        // 将数据转换为字节数组并添加到缓冲区
        appendDataToBuffer(sendBuffer, &data, sizeof(data));

        // 发送数据
        boost::system::error_code ec;
        boost::asio::write(socket, boost::asio::buffer(sendBuffer.data(), sendBuffer.size()), ec);
        std::lock_guard<std::mutex> lock(socket_mutex);
        if (ec)
        {
            std::cerr << "Error while writing: " << ec.message() << std::endl;
            socket.close();
            return;
        }
    }

    void run()
    {
        ros::Rate loop_rate(10);

        while (ros::ok())
        {
            io_service.reset();
            // 获取参数服务器上的指令数据
            int commandDataInt;
            if (nh.getParam("/command_param", commandDataInt))
            {
                uint32_t commandData = static_cast<uint32_t>(commandDataInt);
                sendCommandData(commandData);
            }
            else
            {
                ROS_WARN("Failed to get command data,name: /command_param");
            }
            ros::spinOnce();
            io_service.run_one();
            loop_rate.sleep();
        }
        socket.close();
    }
};

void reconnectSocketThread(VideoSender *sender)
{
    while (ros::ok())
    {

        ROS_INFO("reconnect_theard");
        try
        {

            ROS_INFO("sender->socket.is_open()%d", sender->socket.is_open());
            std::lock_guard<std::mutex> lock(sender->socket_mutex);
            if (!sender->socket.is_open())
            {

                sender->socket = tcp::socket(sender->io_service); // 创建新套接字
                ROS_INFO_STREAM("Reconnecting...to ip " << target_ip << " port " << target_port);

                boost::asio::ip::tcp::resolver::query query(target_ip, std::to_string(target_port));
                boost::asio::ip::tcp::resolver::iterator endpoint_iterator = sender->resolver.resolve(query, sender->ec);
                if (sender->ec)
                {
                    ROS_WARN_STREAM("Resolver error: " << sender->ec.message());

                    continue;
                }
                boost::asio::connect(sender->socket, endpoint_iterator, sender->ec);
                if (sender->ec)
                {
                    sender->socket.close();
                    ROS_WARN_STREAM("Failed to connect: " << sender->ec.message());
                }
                else
                {
                    ROS_INFO("Reconnected successfully");
                }
            }
        }
        catch (std::exception &e)
        {
            ROS_ERROR_STREAM("Exception in reconnectSocketThread: " << e.what());
        }
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 避免频繁尝试重连
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "video_sender");
    ros::NodeHandle nh;
    nh.param<std::string>("target_ip_yaml", target_ip_yaml, "192.168.1.18");
    target_ip = target_ip_yaml;

    nh.param<int>("target_port_yaml", target_port_yaml, 12345);
    target_port = short(target_port_yaml);
    // print target ip and port
    std::cout << "target_ip:" << target_ip << std::endl;
    std::cout << "target_port:" << target_port << std::endl;
    VideoSender sender;
    // sender.run();

    std::thread reconnectThread(reconnectSocketThread, &sender);

    // ROS主循环
    ros::spin();

    // 等待重连线程结束
    reconnectThread.join();
    return 0;
}

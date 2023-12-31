#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/CompressedImage.h"
#include <cv_bridge/cv_bridge.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

using namespace boost::asio;
using boost::system::error_code;

class Session : public boost::enable_shared_from_this<Session> {
public:
    Session(io_service &io_service, ros::NodeHandle &nh)
        : socket_(io_service), nh_(nh) {
        // 获取发送端（客户端）的IP地址

    }
    void setTopics(std::string client_ip) {
        std::string s_client_ip = socket_.remote_endpoint().address().to_string();
        // 构造基于IP的topic名称
        std::replace(s_client_ip.begin(), s_client_ip.end(), '.', '_');
        std::string image_topic = "ip_" + s_client_ip  + "/received_image";
        std::string odometry_topic = "ip_" + s_client_ip + "/received_odometry";
        std::string pointcloud_topic = "ip_" + s_client_ip  + "/received_pointcloud";
        std::string compressed_image_topic = "ip_" + s_client_ip  + "/received_compressed_image";

        // 使用新的topic名称来广告
        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_topic, 1);
        odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic, 1);
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1);
        compressed_image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(compressed_image_topic, 1);  
    }

    ip::tcp::socket &socket() { return socket_; }

    void start() {
        async_read_data();  // Start by reading the combined buffer
    }

private:
    uint32_t dataSize_; // size of incoming data
    uint8_t dataType_;
    uint32_t dataSize;
    std::vector<uchar> header_data_; // Separate buffer for header
    std::vector<uchar> data_;
    std::vector<uchar> point_data_;
    std::vector<uchar> odometry_data_;
    sensor_msgs::PointCloud2 pointcloud2;
    ros::Publisher compressed_image_pub_;
    // std::vector<uchar> dataType_;
    enum { header_length = sizeof(uint32_t), type_length = sizeof(uint8_t)};

    void async_read_data() {
        // dataType_.resize(type_length);
        boost::asio::async_read(socket_, boost::asio::buffer(&dataType_, type_length),
                            boost::bind(&Session::handle_read_type, shared_from_this(),
                                        boost::asio::placeholders::error));
    }

    void handle_read_type(const error_code &error) {
        if (error) {
            // Handle error
            std::cout << "handle_read_type" << std::endl;
            return;
        }
        std::cout << "handle_read_type:" << static_cast<int>(dataType_) << std::endl;
        switch(dataType_) {
            case 0x01: // Image data
                readImageData();
                break;
            case 0x02: // Odometry data
                readOdometryData();
                break;
            case 0x03: // PointCloud2 data
                readPointCloud2Data();
                break;
            case 0x04: // CompressedImage data
                readCompressedImageData();
                break;
            case 0x05: // New case for command data
                readCommandData();
                break;
            default:
                // Handle unknown data type
                break;
        }
    }

    void readCommandData() {
        std::cout << "readCommandData" << std::endl;
        // 假设指令数据结构为：[指令值 (uint32_t)]
        data_.resize(sizeof(uint32_t));
        boost::asio::async_read(socket_, boost::asio::buffer(data_),
            boost::bind(&Session::handle_read_command, shared_from_this(),
                boost::asio::placeholders::error));
    }

    void handle_read_command(const boost::system::error_code& error) {
        if (error) {
            // Handle error
            std::cout << "handle_read_command error" << std::endl;
            async_read_data();
            return;
        }
        std::cout << "handle_read_command" << std::endl;
        uint32_t commandValue = *reinterpret_cast<const uint32_t*>(data_.data());
        // 将指令数据加入到ROS的参数服务器
        int commandDataInt = static_cast<int>(commandValue);
        nh_.setParam("/recieve_command_param", commandDataInt);
        async_read_data(); // Read the next message
    }


    void readCompressedImageData() {
        std::cout << "readCompressedImageData" << std::endl;
        // 首先读取dataSize和formatSize
        header_data_.resize(2 * sizeof(uint32_t)); // For dataSize and formatSize
        boost::asio::async_read(socket_, boost::asio::buffer(header_data_),
            boost::bind(&Session::handle_read_header_compressed_image, shared_from_this(),
                boost::asio::placeholders::error));
    }

    void handle_read_header_compressed_image(const boost::system::error_code& error) {
        if (error) {
            // Handle error
            async_read_data();
            std::cout << "handle_read_header_compressed_image" << std::endl;
            return;
        }
        std::cout << "handle_read_header_compressed_image" << std::endl;
        auto it = header_data_.begin();
        uint32_t dataSize = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);
        uint32_t formatSize = *reinterpret_cast<const uint32_t*>(&(*it));

        // 重新调整data_的大小来读取格式和数据
        data_.resize(formatSize + dataSize);
        boost::asio::async_read(socket_, boost::asio::buffer(data_),
            boost::bind(&Session::handle_read_data_compressed_image, shared_from_this(),
                boost::asio::placeholders::error, formatSize, dataSize));
    }

    void handle_read_data_compressed_image(const boost::system::error_code& error, uint32_t formatSize, uint32_t dataSize) {
        if (error) {
            // Handle error
            async_read_data();
            std::cout << "handle_read_data_compressed_image" << std::endl;
            return;
        }
        std::cout << "handle_read_data_compressed_image" << std::endl;

        sensor_msgs::CompressedImage compressed_image_;
        if (data_.size() != formatSize + dataSize)
            async_read_data();

        auto it = data_.begin();
        for (int i = 0; i < dataSize; i++){
            uint8_t sub_compressed_image_data = *reinterpret_cast<const uint8_t*>(&(*it));
            it += sizeof(uint8_t);
            compressed_image_.data.push_back(sub_compressed_image_data);
        }
        std::cout << "dataSize:" << compressed_image_.data.size() << std::endl;
        it += dataSize;
        
        compressed_image_.format = std::string(it, it + formatSize);
        // compressed_image_.data.assign(it, it + dataSize);

        compressed_image_.header.stamp = ros::Time::now();
        compressed_image_.header.frame_id = "camera_color_optical_frame";

        compressed_image_pub_.publish(compressed_image_);
        async_read_data(); // Read the next message
    }


    void readPointCloud2Data() {
    // 首先，读取header#
    	std::cout << "readPointCloud2data" << std::endl;
        dataSize_ =  
		        sizeof(uint32_t) // seq
                + 2 * sizeof(uint32_t) // stamp.sec, stamp.nsec
                + 2 * sizeof(uint32_t) //height, width
                + sizeof(bool) //bigendian
                + 2 * sizeof(uint32_t) //point_step, row_step
                + sizeof(bool) //is_dense
                //   sizeof(uint32_t) //field size
                + 4 * (2 * sizeof(uint32_t)+sizeof(uint8_t)) //field data
		        + sizeof(uint32_t); //dataSize

        data_.resize(dataSize_);
        boost::asio::async_read(socket_, boost::asio::buffer(data_),
                                boost::bind(&Session::handle_read_data_pointcloud2, shared_from_this(),
                                            boost::asio::placeholders::error));
    }

    void handle_read_data_pointcloud2(const boost::system::error_code& error) {
        if (error) {
            std::cout << "handle_read_data_pointcloud2" << std::endl;
            async_read_data();
            return;
        }
	    std::cout << "handle_read_data_pointcloud2" << std::endl;
        if(data_.size() != dataSize_)
            async_read_data();
        
        pointcloud2.data.clear();
        pointcloud2.fields.clear();

        auto it = data_.begin();
        
	// 读取datasize
        dataSize = *reinterpret_cast<const uint32_t*>(&(*it));
	    std::cout << "pointcloud datasize:" << dataSize << std::endl;
        it += sizeof(dataSize);
	
	// 解析header
        uint32_t seq = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(seq);
        pointcloud2.header.seq = seq;

        uint32_t sec = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(sec);
        uint32_t nsec = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(nsec);
        pointcloud2.header.stamp = ros::Time(sec, nsec);
        pointcloud2.header.frame_id = "camera_init";

    // 解析PointCloud2特定字段
        pointcloud2.height = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(pointcloud2.height);

        pointcloud2.width = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(pointcloud2.width);


        pointcloud2.is_bigendian = *reinterpret_cast<const bool*>(&(*it));
        it += sizeof(pointcloud2.is_bigendian);

        pointcloud2.point_step = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(pointcloud2.point_step);

        pointcloud2.row_step = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(pointcloud2.row_step);

        pointcloud2.is_dense = *reinterpret_cast<const bool*>(&(*it));
        it += sizeof(pointcloud2.is_dense);

        pointcloud2.fields.resize(4);

        pointcloud2.fields[0].name = 'x';

        pointcloud2.fields[0].offset = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);

        pointcloud2.fields[0].datatype = *reinterpret_cast<const uint8_t*>(&(*it));
        it += sizeof(uint8_t);

        pointcloud2.fields[0].count = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);

        pointcloud2.fields[1].name = 'y';

        pointcloud2.fields[1].offset = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);

        pointcloud2.fields[1].datatype = *reinterpret_cast<const uint8_t*>(&(*it));
        it += sizeof(uint8_t);

        pointcloud2.fields[1].count = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);

        pointcloud2.fields[2].name = 'z';

        pointcloud2.fields[2].offset = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);

        pointcloud2.fields[2].datatype = *reinterpret_cast<const uint8_t*>(&(*it));
        it += sizeof(uint8_t);

        pointcloud2.fields[2].count = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);

        pointcloud2.fields[3].name = "intensity";

        pointcloud2.fields[3].offset = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);

        pointcloud2.fields[3].datatype = *reinterpret_cast<const uint8_t*>(&(*it));
        it += sizeof(uint8_t);

        pointcloud2.fields[3].count = *reinterpret_cast<const uint32_t*>(&(*it));
        it += sizeof(uint32_t);

	    std::cout << "pointcloud datasize down" << std::endl;	
        point_data_.resize(dataSize);
        boost::asio::async_read(socket_, boost::asio::buffer(point_data_),
                                boost::bind(&Session::handle_read_more_data_pointcloud2, shared_from_this(),
                                            boost::asio::placeholders::error));
    }

    void handle_read_more_data_pointcloud2(const boost::system::error_code& error){
        std::cout << "handle_read_more_data_pointcloud2" << std::endl;
	    if(dataSize != point_data_.size())
	        async_read_data();

        if (error) {
            // Handle error
            std::cout << "handle_read_more_data_pointcloud2_error" << std::endl;
            async_read_data();
        }
	    auto more_it = point_data_.begin();
        for (int i = 0; i < dataSize; i++){
            uint8_t sub_point_data = *reinterpret_cast<const uint8_t*>(&(*more_it));
            more_it += sizeof(uint8_t);
            pointcloud2.data.push_back(sub_point_data);
        }
        pointcloud_pub_.publish(pointcloud2);
        async_read_data(); // Read the next message
    }


    void readImageData() {
        std::cout << "readImageDate" << std::endl;
        header_data_.resize(header_length);
        boost::asio::async_read(socket_, boost::asio::buffer(header_data_),
                            boost::bind(&Session::header_read_data_image, shared_from_this(),
                                        boost::asio::placeholders::error));
    }

    void header_read_data_image(const error_code &error) {
        if (error) {
            // Handle error
            std::cout << "hander_read_data_image_error" << std::endl;
            async_read_data();
	    return;
        }
        if(header_data_.size() != header_length)
            async_read_data();

        std::cout << "hander_read_data_image" << std::endl;
        dataSize_ = *reinterpret_cast<std::size_t*>(header_data_.data());
        data_.resize(dataSize_);
        boost::asio::async_read(socket_, boost::asio::buffer(data_),
                    boost::bind(&Session::handle_read_data_image, shared_from_this(),
                                boost::asio::placeholders::error));
    }

    void handle_read_data_image(const error_code &error) {
        std::cout << "handle_read_data_image" << std::endl;
        if (!error) {
           // Extract dataSize_ from the first part of the buffer 
            // Ensure the received data size matches expected size (header_length + dataSize_)
            // if (data_.size() != header_length + dataSize_) {
            //     // Handle error: Data received does not match expected size.
            //     return;
            // }
            
            // Decode the image
            if(data_.size() != dataSize_)
                async_read_data();
            cv::Mat image = cv::imdecode(data_, cv::IMREAD_COLOR);
            if(!image.empty()){
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                image_pub_.publish(msg);
            }

            async_read_data(); // go back to reading more data
        } else {
            std::cout << "handle_read_data_image" << std::endl;
            // You may want to handle errors here
        }
    }

    void readOdometryData() {
    odometry_data_.resize(13 * sizeof(double) + 2 * sizeof(uint32_t));
    boost::asio::async_read(socket_, boost::asio::buffer(odometry_data_),
                    boost::bind(&Session::handle_read_data_odometry, shared_from_this(),
                                boost::asio::placeholders::error));
    }   

    void handle_read_data_odometry(const error_code &error) {
        if (error) {
            // Handle error
            std::cout << "handle_read_data_odometry" << std::endl;
            return;
        }

        const uchar* recvBuffer = odometry_data_.data();

        size_t idx = 0;

        uint32_t sec = *reinterpret_cast<const uint32_t*>(&recvBuffer[idx]);
        idx += sizeof(uint32_t);
        uint32_t nsec = *reinterpret_cast<const uint32_t*>(&recvBuffer[idx]);
        idx += sizeof(uint32_t);
        // 1. 解码位置数据
        double x = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double y = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double z = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);

        // 2. 解码方向（四元数）数据
        double qx = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double qy = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double qz = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double qw = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);

        // 3. 解码线速度
        double vx = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double vy = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double vz = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);

        // 4. 解码角速度
        double wx = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double wy = *reinterpret_cast<const double*>(&recvBuffer[idx]);
        idx += sizeof(double);
        double wz = *reinterpret_cast<const double*>(&recvBuffer[idx]);

        // 构建Odometry消息
        nav_msgs::Odometry odom;
        odom.header.stamp.sec = sec;
        odom.header.stamp.nsec = nsec;
        // odom.header.stamp = ros::Time::now();
        // 这里可以设置frame_id，例如odom.header.frame_id = "odom";
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = z;
        odom.pose.pose.orientation.x = qx;
        odom.pose.pose.orientation.y = qy;
        odom.pose.pose.orientation.z = qz;
        odom.pose.pose.orientation.w = qw;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = vz;
        odom.twist.twist.angular.x = wx;
        odom.twist.twist.angular.y = wy;
        odom.twist.twist.angular.z = wz;

        // Publish Odometry message
        odometry_pub_.publish(odom);

        async_read_data(); // go back to reading more data
    }



    ip::tcp::socket socket_;
    ros::NodeHandle &nh_;
    ros::Publisher image_pub_;
    ros::Publisher odometry_pub_;
    ros::Publisher pointcloud_pub_;
    enum { max_length = 1000000 + header_length };  // Updated max_length
};



typedef boost::shared_ptr<Session> SessionPtr;

class Server {
public:
    Server(io_service &io_service, short port, ros::NodeHandle &nh)
        : io_service_(io_service), acceptor_(io_service, ip::tcp::endpoint(ip::tcp::v4(), port)), nh_(nh) {
        start_accept();
    }

private:
    void start_accept() {
        SessionPtr new_session(new Session(io_service_, nh_));
        acceptor_.async_accept(new_session->socket(),
                               boost::bind(&Server::handle_accept, this, new_session,
                                           boost::asio::placeholders::error));
    }

    void handle_accept(SessionPtr new_session, const error_code &error) {
        if (!error) {
            std::string s_client_ip  = new_session->socket().remote_endpoint().address().to_string();
            new_session->setTopics(s_client_ip );
            boost::thread new_thread(boost::bind(&Session::start, new_session));
            new_thread.detach();
        }
        start_accept();
    }

    io_service &io_service_;
    ip::tcp::acceptor acceptor_;
    ros::NodeHandle &nh_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "boost_server_node");
    ros::NodeHandle nh;

    int port;
    nh.param<int>("server_port", port, 12345);

    try {
        io_service io_service;
        Server server(io_service, port, nh);
        while (ros::master::check() && ros::ok()){
            io_service.poll();
        }
        
    } catch (std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    return 0;
}

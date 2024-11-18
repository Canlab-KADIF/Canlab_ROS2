#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode() : Node("listener")
    {
        // '/sensing/lidar/top/rectified/pointcloud' 토픽을 구독합니다.
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/top/rectified/pointcloud", 10, std::bind(&ListenerNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        /* 수신한 PointCloud2 메시지를 처리하고 정보를 출력
        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message with width: %d, height: %d", msg->width, msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        if (msg->width > 0 && msg->height > 0)
        {
            RCLCPP_INFO(this->get_logger(), "First point (x, y, z): (%f, %f, %f)", *iter_x, *iter_y, *iter_z);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "PointCloud2 message contains no data.");
        }
        */
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


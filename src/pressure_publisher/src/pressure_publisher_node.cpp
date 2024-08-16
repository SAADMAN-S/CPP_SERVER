#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "pressure_publisher_node");
    ros::NodeHandle nh;

    // Create a publisher for pressure data
    ros::Publisher pressure_pub = nh.advertise<std_msgs::Float32MultiArray>("pressure_data", 10);

    ros::Rate loop_rate(1); // Publish at 1 Hz

    // Seed random number generator
    std::srand(std::time(0));

    while (ros::ok()) {
        std_msgs::Float32MultiArray pressure_msg;
        pressure_msg.data.resize(5);

        for (size_t i = 0; i < 5; i++) {
            pressure_msg.data[i] = static_cast<float>(std::rand() % 1000) / 10.0; // Generate values between 0.0 and 99.9
        }

        pressure_pub.publish(pressure_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

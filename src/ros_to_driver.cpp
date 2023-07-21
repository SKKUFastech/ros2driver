// Include ROS2 C++ dependencies
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
//
#include <stdlib.h>
#include <string.h>
// dependencies for udp
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define SERVER_IP "192.168.0.2" // Replace with the server's IP address
#define BUFFER_SIZE 1024
#define PORT 3001 // UDP Port Number of Ezi-SERVO2 Plus-E

int udpClientSocket;
struct sockaddr_in serverAddr;

// Subscriber callback function
void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // This function will be called whenever a new message is received on the "cmd_vel" topic
    // Example: Sending linear and angular velocity over UDP to the server
    float linear_vel = msg->linear.x;
    float angular_vel = msg->angular.z;

    printf("lv: %lf, av: %lf\n",linear_vel,angular_vel);
    // Convert float values to char buffer before sending
    char buffer[258];
    // snprintf(buffer, sizeof(buffer), "Linear Velocity: %f, Angular Velocity: %f", linear_vel, angular_vel);
    buffer[0] = 0xAA;
    buffer[1] = 0x04;
    buffer[2] = 0x38;
    buffer[3] = 0x00;
    buffer[4] = 0x2A;
    buffer[5] = 0x00;
    // Send the data over the UDP socket
    sendto(udpClientSocket, buffer, 6, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

    // Receive message from server
    int len = recvfrom(udpClientSocket, (char *)buffer, BUFFER_SIZE, 0, NULL, NULL);
    buffer[len] = '\0';
    printf("Server: %s\n", buffer);
}

int main(int argc, char **argv)
{

    udpClientSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpClientSocket < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("cmd_vel_subscriber"), "Error opening UDP client socket");
        return -1;
    }

    // Set up the server address (replace SERVER_IP and SERVER_PORT with the appropriate values)
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0) {
            perror("inet_pton failed");
            exit(EXIT_FAILURE);
        }
    // Initialize ROS2 node
    rclcpp::init(argc, argv);

    // Create a ROS2 node with a unique name
    auto node = rclcpp::Node::make_shared("cmd_vel_subscriber_node");

    // Create a subscriber object for the "cmd_vel" topic
    auto subscriber = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, cmdVelCallback);

    // Spin the node to start processing incoming messages
    rclcpp::spin(node);

    // Shut down ROS2 node
    rclcpp::shutdown();
    // Close the UDP socket
    close(udpClientSocket);
    return 0;
}

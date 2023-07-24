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
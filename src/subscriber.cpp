// Include ROS2 C++ dependencies
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ros2driver/msg/servoenable42.hpp>
#include <ros2driver/msg/movestop49.hpp>
#include <ros2driver/msg/emergencystop50.hpp>
#include <ros2driver/msg/movesingleaxisabspos52.hpp>
//
#include <stdlib.h>
#include <string.h>
#include <stdint.h> // For uint64_t
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

// Function of decimal to hexadecimal
void decimalToHex(int decimalValue, unsigned char* hexArray) {
    hexArray[0] = (decimalValue >> 24) & 0xFF;
    hexArray[1] = (decimalValue >> 16) & 0xFF;
    hexArray[2] = (decimalValue >> 8) & 0xFF;
    hexArray[3] = decimalValue & 0xFF;
}
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

void ServoEnable_cb(const ros2driver::msg::Servoenable42 &msg)
{
    int power = msg.power;
    RCLCPP_INFO(rclcpp::get_logger("ServoEnable_sub"), "Power: %d", msg.power);
    char buffer[258];

    if (power == 0){
        buffer[0] = 0xAA;
        buffer[1] = 0x04;
        buffer[2] = 0x38;
        buffer[3] = 0x00;
        buffer[4] = 0x2A;
        buffer[5] = 0x00;
        // Send the data over the UDP socket
        sendto(udpClientSocket, buffer, 6, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        printf("OFF\n");
        // Receive message from server
        int len = recvfrom(udpClientSocket, (char *)buffer, BUFFER_SIZE, 0, NULL, NULL);
        buffer[len] = '\0';
        printf("Server: ");
        for (ssize_t i = 0; i < len; i++) {
            printf("%02x ", (unsigned char)buffer[i]);
        }
        printf("\n");
    }
    else if (power == 1){
        buffer[0] = 0xAA;
        buffer[1] = 0x04;
        buffer[2] = 0x38;
        buffer[3] = 0x00;
        buffer[4] = 0x2A;
        buffer[5] = 0x01;
        // Send the data over the UDP socket
        sendto(udpClientSocket, buffer, 6, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        printf("ON\n");
        // Receive message from server
        int len = recvfrom(udpClientSocket, (char *)buffer, BUFFER_SIZE, 0, NULL, NULL);
        buffer[len] = '\0';
        printf("Server: ");
        for (ssize_t i = 0; i < len; i++) {
            printf("%02x ", (unsigned char)buffer[i]);
        }  
        printf("\n");  
    }
    else{
        printf("WRONG SIGNAL\n");
    }
}

void MoveStop_cb(const ros2driver::msg::Movestop49 &msg)
{
    int movestop = msg.movestop;
    RCLCPP_INFO(rclcpp::get_logger("MoveStop_sub"), "Movestop: %d", movestop);
    char buffer[258];

    if (movestop == 0){
        buffer[0] = 0xAA;
        buffer[1] = 0x03;
        buffer[2] = 0x49;
        buffer[3] = 0x00;
        buffer[4] = 0x31;
        // Send the data over the UDP socket
        sendto(udpClientSocket, buffer, 5, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        printf("MOVESTOP\n");
        // Receive message from server
        int len = recvfrom(udpClientSocket, (char *)buffer, BUFFER_SIZE, 0, NULL, NULL);
        buffer[len] = '\0';
        printf("Server: ");
        for (ssize_t i = 0; i < len; i++) {
            printf("%02x ", (unsigned char)buffer[i]);
        }
        printf("\n");
    }
    else{
        printf("WRONG SIGNAL\n");
    }
}

void EmergencyStop_cb(const ros2driver::msg::Emergencystop50 &msg)
{
    int emgstop = msg.emgstop;
    RCLCPP_INFO(rclcpp::get_logger("EmergencyStop_sub"), "Emergencystop: %d", emgstop);
    char buffer[258];

    if (emgstop == 0){
        buffer[0] = 0xAA;
        buffer[1] = 0x03;
        buffer[2] = 0x50;
        buffer[3] = 0x00;
        buffer[4] = 0x32;
        // Send the data over the UDP socket
        sendto(udpClientSocket, buffer, 5, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        printf("EMERGENCYSTOP\n");
        // Receive message from server
        int len = recvfrom(udpClientSocket, (char *)buffer, BUFFER_SIZE, 0, NULL, NULL);
        buffer[len] = '\0';
        printf("Server: ");
        for (ssize_t i = 0; i < len; i++) {
            printf("%02x ", (unsigned char)buffer[i]);
        }
        printf("\n");
    }
    else{
        printf("WRONG SIGNAL\n");
    }
}

void MoveSingleAxisAbsPos_cb(const ros2driver::msg::Movesingleaxisabspos52 &msg)
{
    int position = msg.position;
    int speed = msg.speed;
    unsigned char positionBuffer[4];
    unsigned char speedBuffer[4];
    int flag = 0;
    RCLCPP_INFO(rclcpp::get_logger("MoveSingleAxisAbsPos_sub"), "Postion, Speed: %d, %d",position, speed);
    char buffer[258];

    if (position >= -134217728 && position <= 134217727){
        // Convert power to hex and store it in the buffer
        decimalToHex(position, positionBuffer);
        flag = 1;
    }
    if (speed >= 0 && speed <= 2500000){
        // Convert power to hex and store it in the buffer
        decimalToHex(speed, speedBuffer);
        flag = 1;
    }

    if(flag ==1){
        buffer[0] = 0xAA;
        buffer[1] = 0x0B;
        buffer[2] = 0x52;
        buffer[3] = 0x00;
        buffer[4] = 0x34;
        buffer[5] = positionBuffer[3];
        buffer[6] = positionBuffer[2];
        buffer[7] = positionBuffer[1];
        buffer[8] = positionBuffer[0];
        buffer[9] = speedBuffer[3];
        buffer[10] = speedBuffer[2];
        buffer[11] = speedBuffer[1];
        buffer[12] = speedBuffer[0];

        // Send the data over the UDP socket
        sendto(udpClientSocket, buffer, 13, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
        printf("MOVING\n");

        for(int j=0; j<13; j++){
            printf("%02x ", (unsigned char)buffer[j]);
        }
        printf("\n");
        // Receive message from server
        int len = recvfrom(udpClientSocket, (char *)buffer, BUFFER_SIZE, 0, NULL, NULL);
        buffer[len] = '\0';
        printf("Server: ");
        for (ssize_t i = 0; i < len; i++) {
            printf("%02x ", (unsigned char)buffer[i]);
        }
        printf("\n");
    }
    else{
        printf("INVALID POSITION OR SPEED");
    }
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
    auto node = rclcpp::Node::make_shared("subscriber_set_node");

    // Create a subscriber object for the "cmd_vel" topic
    auto subscriber = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, cmdVelCallback);
    auto Servoenable_sub = node->create_subscription<ros2driver::msg::Servoenable42>(
        "ServoEnable", 1, ServoEnable_cb);
    auto MoveStop_sub = node->create_subscription<ros2driver::msg::Movestop49>(
        "MoveStop", 1, MoveStop_cb);
    auto EmergencyStop_sub = node->create_subscription<ros2driver::msg::Emergencystop50>(
        "EmergencyStop", 1, EmergencyStop_cb);
    auto MoveSingleAxisAbsPos_sub = node->create_subscription<ros2driver::msg::Movesingleaxisabspos52>(
        "MoveSingleAxisAbsPos", 1, MoveSingleAxisAbsPos_cb);
    // Spin the node to start processing incoming messages
    rclcpp::spin(node);

    // Shut down ROS2 node
    rclcpp::shutdown();
    // Close the UDP socket
    close(udpClientSocket);
    return 0;
}

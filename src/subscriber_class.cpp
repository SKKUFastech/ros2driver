#include <rclcpp/rclcpp.hpp>
#include <ros2driver/msg/servoenable42.hpp>
#include <ros2driver/msg/movestop49.hpp>
#include <ros2driver/msg/emergencystop50.hpp>
#include <ros2driver/msg/movesingleaxisabspos52.hpp>
#include <ros2driver/msg/movesingleaxisincpos53.hpp>
#include <ros2driver/msg/movetolimit54.hpp>
#include <ros2driver/msg/movevelocity55.hpp>
#include <ros2driver/msg/positionabsoverride56.hpp>
#include <ros2driver/msg/movepause88.hpp>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define SERVER_IP "192.168.0.2"  //192.168.0.171
#define BUFFER_SIZE 1024
#define PORT 3001

class UDPServoController : public rclcpp::Node {
public:
    UDPServoController() : Node("subscriber_class_node") {
        // Create the UDP socket
        udpClientSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if (udpClientSocket < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error opening UDP client socket");
            return;
        }

        // Set up the server address
        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(PORT);
        if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0) {
            perror("inet_pton failed");
            exit(EXIT_FAILURE);
        }
        
        this->ServoEnable_sub = this->create_subscription<ros2driver::msg::Servoenable42>(
            "ServoEnable", 1, std::bind(&UDPServoController::ServoEnable_cb, this, std::placeholders::_1));
        // Add other subscribers for other topics as needed
        this->MoveStop_sub = this->create_subscription<ros2driver::msg::Movestop49>(
            "MoveStop", 1, std::bind(&UDPServoController::MoveStop_cb, this, std::placeholders::_1));
        this-> EmergencyStop_sub = this->create_subscription<ros2driver::msg::Emergencystop50>(
            "EmergencyStop", 1, std::bind(&UDPServoController::EmergencyStop_cb, this, std::placeholders::_1));
        this->MoveSingleAxisAbsPos_sub = this->create_subscription<ros2driver::msg::Movesingleaxisabspos52>(
            "MoveSingleAxisAbsPos", 1, std::bind(&UDPServoController::MoveSingleAxisAbsPos_cb, this, std::placeholders::_1));
        this->MoveSingleAxisIncPos_sub = this->create_subscription<ros2driver::msg::Movesingleaxisincpos53>(
            "MoveSingleAxisIncPos", 1, std::bind(&UDPServoController::MoveSingleAxisIncPos_cb, this, std::placeholders::_1));
        this->MoveToLimit_sub = this->create_subscription<ros2driver::msg::Movetolimit54>(
            "MoveToLimit", 1, std::bind(&UDPServoController::MovetoLimit_cb, this, std::placeholders::_1));
        this->MoveVelocity_sub = this->create_subscription<ros2driver::msg::Movevelocity55>(
            "MoveVelocity", 1, std::bind(&UDPServoController::MoveVelocity_cb, this, std::placeholders::_1));
        this->PositionAbsOverride_sub = this->create_subscription<ros2driver::msg::Positionabsoverride56>(
            "PositionAbsOverride", 1, std::bind(&UDPServoController::PositionAbsOverride_cb, this, std::placeholders::_1));
        this->MovePause_sub = this->create_subscription<ros2driver::msg::Movepause88>(
            "MovePause", 1, std::bind(&UDPServoController::MovePause_cb, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "UDPServoController initialized.");
    }

    void closesocket(){
            close(this->udpClientSocket);
    }

private:
    int udpClientSocket;
    struct sockaddr_in serverAddr;

    rclcpp::Subscription<ros2driver::msg::Servoenable42>::SharedPtr ServoEnable_sub;
    rclcpp::Subscription<ros2driver::msg::Movestop49>::SharedPtr MoveStop_sub;
    rclcpp::Subscription<ros2driver::msg::Emergencystop50>::SharedPtr EmergencyStop_sub;
    rclcpp::Subscription<ros2driver::msg::Movesingleaxisabspos52>::SharedPtr MoveSingleAxisAbsPos_sub;
    rclcpp::Subscription<ros2driver::msg::Movesingleaxisincpos53>::SharedPtr MoveSingleAxisIncPos_sub;
    rclcpp::Subscription<ros2driver::msg::Movetolimit54>::SharedPtr MoveToLimit_sub;
    rclcpp::Subscription<ros2driver::msg::Movevelocity55>::SharedPtr MoveVelocity_sub;
    rclcpp::Subscription<ros2driver::msg::Positionabsoverride56>::SharedPtr PositionAbsOverride_sub;
    rclcpp::Subscription<ros2driver::msg::Movepause88>::SharedPtr MovePause_sub;

    //decimal(10진수) -> hexadecimal(16진수)
    void decimalToHex(int decimalValue, unsigned char* hexArray, int len) {
    int i=0;
    for(i=0;i<len;i++){
        if(i!=(len-1)){
            hexArray[i]=(decimalValue >> 8*(len-1-i)) & 0xFF;
        }
        else{
            hexArray[i]=decimalValue & 0xFF;
        }
    }
}
    // Add other subscribers for other topics as needed

    // Callback for ServoEnable topic
    void ServoEnable_cb(const ros2driver::msg::Servoenable42::SharedPtr msg) {
        int power = msg->power;
        RCLCPP_INFO(rclcpp::get_logger("ServoEnable_sub"), "Power: %d", power);
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

    // Add other callback functions for other topics as needed
    void MoveStop_cb(const ros2driver::msg::Movestop49::SharedPtr msg)
    {
        int movestop = msg->movestop;
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

    void EmergencyStop_cb(const ros2driver::msg::Emergencystop50::SharedPtr msg)
    {
        int emgstop = msg->emgstop;
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

    void MoveSingleAxisAbsPos_cb(const ros2driver::msg::Movesingleaxisabspos52::SharedPtr msg)
    {
        int position = msg->position;
        int speed = msg->speed;
        unsigned char positionBuffer[4];
        unsigned char speedBuffer[4];
        int flag = 0;
        RCLCPP_INFO(rclcpp::get_logger("MoveSingleAxisAbsPos_sub"), "Postion, Speed: %d, %d",position, speed);
        char buffer[258];

        if (position >= -134217728 && position <= 134217727){
            // Convert power to hex and store it in the buffer
            decimalToHex(position, positionBuffer, 4);
            flag = 1;
        }
        if (speed >= 0 && speed <= 2500000){
            // Convert power to hex and store it in the buffer
            decimalToHex(speed, speedBuffer, 4);
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

    void MoveSingleAxisIncPos_cb(const ros2driver::msg::Movesingleaxisincpos53::SharedPtr msg)
    {
        int position = msg->position;
        int speed = msg->speed;
        unsigned char positionBuffer[4];
        unsigned char speedBuffer[4];
        int flag = 0;
        RCLCPP_INFO(rclcpp::get_logger("MoveSingleAxisIncPos_sub"), "Postion, Speed: %d, %d",position, speed);
        char buffer[258];

        if (position >= -134217728 && position <= 134217727){
            // Convert power to hex and store it in the buffer
            decimalToHex(position, positionBuffer, 4);
            flag = 1;
        }
        if (speed >= 0 && speed <= 2500000){
            // Convert power to hex and store it in the buffer
            decimalToHex(speed, speedBuffer, 4);
            flag = 1;
        }

        if(flag ==1){
            buffer[0] = 0xAA;
            buffer[1] = 0x0B;
            buffer[2] = 0x53;
            buffer[3] = 0x00;
            buffer[4] = 0x35;
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

    void MovetoLimit_cb(const ros2driver::msg::Movetolimit54::SharedPtr msg)
    {
        int speed = msg->speed;
        int limit= msg->limit;
        
        unsigned char speedBuffer[4];
        
        
        RCLCPP_INFO(rclcpp::get_logger("MovetoLimit_sub"), "speed: %d, limit: %d", speed, limit);
        char buffer[258];

        if (speed>0 && speed<35000){
            if(limit==0 || limit==1){
                decimalToHex(speed, speedBuffer, 4);
                
                buffer[0] = 0xAA;
                buffer[1] = 0x08;
                buffer[2] = 0x54;
                buffer[3] = 0x00;
                buffer[4] = 0x36;
                buffer[5] = speedBuffer[3];
                buffer[6] = speedBuffer[2];
                buffer[7] = speedBuffer[1];
                buffer[8] = speedBuffer[0];
                if(limit==0){
                    buffer[9]=0x00;
                }
                else{
                    buffer[9]=0x01;
                }
                // Send the data over the UDP socket
                sendto(udpClientSocket, buffer, 10, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
                printf("MOVELIMIT\n");
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
        else{
            printf("WRONG SIGNAL\n");
        }
    }

    void MoveVelocity_cb(const ros2driver::msg::Movevelocity55::SharedPtr msg)
    {
        int speed = msg->speed;
        int jog=msg->jog;
        
        unsigned char speedBuffer[4];
        
        
        RCLCPP_INFO(rclcpp::get_logger("MoveVelocity_sub"), "speed: %d, jog: %d", speed, jog);
        char buffer[258];

        if (speed>0 && speed<35000){
            if(jog==0 || jog ==1){
                decimalToHex(speed, speedBuffer, 4);
                
                buffer[0] = 0xAA;
                buffer[1] = 0x08;
                buffer[2] = 0x55;
                buffer[3] = 0x00;
                buffer[4] = 0x37;
                buffer[5] = speedBuffer[3];
                buffer[6] = speedBuffer[2];
                buffer[7] = speedBuffer[1];
                buffer[8] = speedBuffer[0];
                if(jog==0){
                    buffer[9]=0x00;
                }
                else{
                    buffer[9]=0x01;
                }
                // Send the data over the UDP socket
                sendto(udpClientSocket, buffer, 10, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
                printf("MOVEVELOCITY\n");
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
        else{
            printf("WRONG SIGNAL\n");
        }
    }

    void PositionAbsOverride_cb(const ros2driver::msg::Positionabsoverride56::SharedPtr msg)
    {
        int target_abs_pos = msg->target_abs_pos;
        
        unsigned char positionBuffer[4];
        
        RCLCPP_INFO(rclcpp::get_logger("PositionAbsOverride_sub"), "target_abs_pos: %d",target_abs_pos);
        char buffer[258];

        if (target_abs_pos >= -134217728 && target_abs_pos <= 134217727){
            // Convert power to hex and store it in the buffer
            decimalToHex(target_abs_pos, positionBuffer, 4);

            buffer[0] = 0xAA;
            buffer[1] = 0x07;
            buffer[2] = 0x53;
            buffer[3] = 0x56;
            buffer[4] = 0x38;
            buffer[5] = positionBuffer[3];
            buffer[6] = positionBuffer[2];
            buffer[7] = positionBuffer[1];
            buffer[8] = positionBuffer[0];

            // Send the data over the UDP socket
            sendto(udpClientSocket, buffer, 9, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
            printf("Change abs pos target\n");

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
            printf("INVALID POSITION");
        }
    }

    void MovePause_cb(const ros2driver::msg::Movepause88::SharedPtr msg)
    {
        int moveps = msg->moveps;
        RCLCPP_INFO(rclcpp::get_logger("MovePause_sub"), "MovePause: %d", moveps);
        char buffer[258];

        if (moveps == 0){
            buffer[0] = 0xAA;
            buffer[1] = 0x04;
            buffer[2] = 0x88;
            buffer[3] = 0x00;
            buffer[4] = 0x58;
            buffer[5] = 0x00;
            // Send the data over the UDP socket
            sendto(udpClientSocket, buffer, 6, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
            printf("RESUME\n");
            // Receive message from server
            int len = recvfrom(udpClientSocket, (char *)buffer, BUFFER_SIZE, 0, NULL, NULL);
            buffer[len] = '\0';
            printf("Server: ");
            for (ssize_t i = 0; i < len; i++) {
                printf("%02x ", (unsigned char)buffer[i]);
            }
            printf("\n");
        }
        else if (moveps == 1){
            buffer[0] = 0xAA;
            buffer[1] = 0x04;
            buffer[2] = 0x88;
            buffer[3] = 0x00;
            buffer[4] = 0x58;
            buffer[5] = 0x01;
            // Send the data over the UDP socket
            sendto(udpClientSocket, buffer, 6, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
            printf("PAUSE\n");
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

};

int main(int argc, char **argv) {
    // Initialize ROS2 node
    rclcpp::init(argc, argv);

    // Create and run the UDPServoController node
    auto node = std::make_shared<UDPServoController>();
    rclcpp::spin(node);

    // Close the UDP socket
    // close(node->udpClientSocket);
    node->closesocket();

    // Shut down ROS2 node
    rclcpp::shutdown();
    return 0;
}
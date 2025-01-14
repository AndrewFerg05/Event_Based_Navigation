/*
Filename    : Software/src/Communication.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Communication code for communicating with base station and Arduino
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------

// Local
#include "Communication.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------
#define MAX_PACKET_SIZE 65507           // Max packet in bytes for UDP
#define PC_IP           "192.168.0.2"   // Change to base station IP
#define PC_PORT         5005            // Application address for base station
//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
int iterations = 0;


//==============================================================================
// Functions
//------------------------------------------------------------------------------
void CM_loop(
    std::atomic<ThreadState>& data_sync_state,
    std::atomic<ThreadState>& frontend_state,
    std::atomic<ThreadState>& backend_state,
    interface_DA_to_FE* data_DA,
    interface_FE_to_BE* data_FE) {

    auto start_time = std::chrono::steady_clock::now();   

    std::uint8_t command = 100; //Get this from external source

    int frame_id = 3;
    cv::Mat frame = cv::imread("C:/Users/pokew/Documents/Year5/Project/example.jpg");
    if (frame.empty()) {
        std::cerr << "Failed to load image." << std::endl;
        command = 0;
    }

    int bufferSize = 0;

    while (true) {

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

        // AF - Test Thread Control
        if (elapsed > 1)
        {
            if (elapsed > 15)
            {
                command = 1;
            }
            else 
            {
                command = 0;
            }
        }

        // Thread control
        if (command == 0) {
            data_sync_state = ThreadState::Running;
            frontend_state = ThreadState::Running;
            backend_state = ThreadState::Running;

            // Arduino Communication
            //      Receive from arduino control instructions
                

            //      Transmit to arduino displacement estimates
            //          Get pose / displacement from BE



            // Base Station Communication
            //      Get frames from DA and transmit on UDP
            bufferSize = data_DA->checkBuffer();
            if (bufferSize > 0 && data_DA->checkIndex('C') < bufferSize)
            {
                std::cout << "DA: " << data_DA->readBuffer('C') << std::endl;
                C_transmit_frame(frame, frame_id);
            }
            else
            {
                std::cout << "DA Buffer Empty!" << std::endl;
            }

            //      Get event frames from FE and transmit on UDP
            bufferSize = data_FE->checkBuffer();
            if (bufferSize > 0 && data_FE->checkIndex('C') < bufferSize)
            {
                std::cout << "FE: " << data_FE->readBuffer('C') << std::endl;
                //Transmit event frame
            }
            else
            {
                std::cout << "FE Buffer Empty!" << std::endl;
            }
            
            sleep_ms(100);
            
        } else if (command == 1) {
            std::cout << "Comms Stopping" << std::endl;
            data_sync_state = ThreadState::Stopped;
            frontend_state = ThreadState::Stopped;
            backend_state = ThreadState::Stopped;
            break;
        } else if (command == 2) {
            data_sync_state = ThreadState::Paused;
            frontend_state = ThreadState::Paused;
            backend_state = ThreadState::Paused;
        } else if (command == 3) {
            data_sync_state = ThreadState::Reset;
            frontend_state = ThreadState::Reset;
            backend_state = ThreadState::Reset;
        } else if (command == 4) {
            std::cout << "Comms Testing" << std::endl;
            data_sync_state = ThreadState::Test;
            frontend_state = ThreadState::Test;
            backend_state = ThreadState::Test;
            sleep_ms(100);
        }
        else if (command == 100){
            std::cout << "Unknown state" << std::endl;
            sleep_ms(100);
        }
    }
}


void C_transmit_frame(cv::Mat frame, int frame_id) {

    if (initNet() != 0) 
    {
        std::cerr << "WSAStartup failed!" << std::endl;
        return;
    }

    // Serialize the frame
    std::vector<uchar> encoded_frame;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};

    if (!cv::imencode(".jpg", frame, encoded_frame, params)) {
        std::cerr << "Failed to encode the frame." << std::endl;
        return;
    }

    // Prepare header and data
    uint32_t frame_size = encoded_frame.size();
    uint32_t frame_id_le = little_endian(frame_id);  // Ensure little-endian format
    uint32_t frame_size_le = little_endian(frame_size);

    std::cout << "Sending frame ID: " << frame_id << ", size: " << frame_size << " bytes" << std::endl;

    // Allocate a buffer for the header and frame data
    uchar* send_buffer = static_cast<uchar*>(malloc(8 + frame_size));
    if (!send_buffer) {
        std::cerr << "Memory allocation failed." << std::endl;
        return;
    }

    // Copy the header (frame ID and size) into the buffer
    memcpy(send_buffer, &frame_id_le, 4);
    memcpy(send_buffer + 4, &frame_size_le, 4);

    // Copy the encoded frame data into the buffer
    memcpy(send_buffer + 8, encoded_frame.data(), frame_size);

    // Create a UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        free(send_buffer);
        return;
    }

    // Configure the server address
    struct sockaddr_in server_address;
    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PC_PORT);
    inet_pton(AF_INET, PC_IP, &server_address.sin_addr);

    // Send the data in chunks
    size_t total_size = 8 + frame_size;  // Header (8 bytes) + frame data
    uchar* data_ptr = send_buffer;

    for (size_t i = 0; i < total_size; i += MAX_PACKET_SIZE) 
    {
        size_t chunk_size = (i + MAX_PACKET_SIZE < total_size) ? MAX_PACKET_SIZE : (total_size - i);
        
        // Cast uchar* to const char* explicitly for the sendto function
        if (sendto(sockfd, reinterpret_cast<const char*>(data_ptr + i), chunk_size, 0, 
                (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
            perror("Failed to send data chunk");
            break;
        }
    }
    // Clean up
    close(sockfd);
    free(send_buffer);
    cleanupNet();
}

int initNet() 
{
    #ifdef _WIN32
        WSADATA wsaData;
        return WSAStartup(MAKEWORD(2, 2), &wsaData);
    #else
        return 0;
    #endif
}

void cleanupNet() 
{
    #ifdef _WIN32
        WSACleanup();
    #endif
}

//==============================================================================
// End of File : Software/src/Communication.cpp

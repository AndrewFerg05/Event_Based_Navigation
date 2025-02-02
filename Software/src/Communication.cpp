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

// Control Signals
#define RUN     0
#define STOP    1
#define PAUSE   2
#define RESET   3
#define TEST    4

#define TEST_IMAGE  "../example.jpg"
#define TEST_RUN_TIME 20

#define MAX_PACKET_SIZE 65507            // Max packet in bytes for UDP
#define PC_IP           "192.168.43.245" // Change to base station IP (SARK's laptop)
#define PC_PORT         5005             // Application address for base station
#define ID_FRAME        0
#define ID_EVENT        1
#define ID_STATUS       2
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
    ThreadSafeFIFO<InputDataSync>* data_DA,
    CommunicationManager* comms,
    CM_serialInterface* serial) {

    auto start_time = std::chrono::steady_clock::now();   

    std::uint8_t commandReceived = 100; //Get this from external serial source
    std::uint8_t command = 100; //Get this from external source

    bool state_change_called = false; //Used to only set the atomics once
	
    int frameId = 3;
    cv::Mat frame = cv::imread(TEST_IMAGE);
    if (frame.empty()) {
        std::cerr << "Failed to load image." << std::endl;
        command = 0;
    }

    int bufferSize = 0;
    std::optional<int> last_output;
    while (true) {

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

        // AF - Test Thread Control
        if (elapsed > 1)
        {
            if (elapsed > TEST_RUN_TIME)
            {
                command = STOP;
                state_change_called = true;
            }
            else 
            {
                command = RUN;
                state_change_called = true;
            }
        }

        // Thread control
        if (command == RUN) {

            if(state_change_called){
            data_sync_state = ThreadState::Running;
            frontend_state = ThreadState::Running;
            backend_state = ThreadState::Running;
            state_change_called = false;
            }

            // Arduino Communication
            commandReceived = CM_serialReceive(serial);
            if (commandReceived != 42){     // If not no response
                command = commandReceived;
            }

            //      Send to ESP32 displacement estimates
            //          Get pose / displacement from BE
            CM_serialSendStatus(serial, 3, 4);


            // Base Station Communication
            //      Get frames from DA and transmit on UDP
            if(!comms->processQueues())
            {
                std::cout << "No data To Send" << std::endl;
            }
            CM_transmitFrame(frame, 0);

            //      Get event frames from FE and transmit on UDP
            CM_transmitFrame(frame, 1);

            //      Get position from BE and transmit on UDP
            CM_transmitStatus(3,4,5,6);
            
            sleep_ms(10);
            
        } else if (command == STOP) {
            // Stop Condition
            if(state_change_called){
            data_sync_state = ThreadState::Stopped;
            frontend_state = ThreadState::Stopped;
            backend_state = ThreadState::Stopped;
            state_change_called = false;
            }

            data_DA->stop_queue();  //Wake FE if waiting on data
            break;

        } else if (command == PAUSE) {
            // Pause Condition
           if(state_change_called){
            data_sync_state = ThreadState::Paused;
            frontend_state = ThreadState::Paused;
            backend_state = ThreadState::Paused;
            state_change_called = false;
            }

        } else if (command == RESET) {
            // Reset Condition
           if(state_change_called){
            data_sync_state = ThreadState::Reset;
            frontend_state = ThreadState::Reset;
            backend_state = ThreadState::Reset;
            state_change_called = false;
            }

        } else if (command == TEST) {
            // Testing Conditionn
          if(state_change_called){
            data_sync_state = ThreadState::Test;
            frontend_state = ThreadState::Test;
            backend_state = ThreadState::Test;
            state_change_called = false;
            }
            std::cout << "Comms Testing" << std::endl;
            sleep_ms(100);
        }
        else if (command == 100){
            std::cout << "Unknown state" << std::endl;
            sleep_ms(100);
        }
    }
}

std::uint8_t CM_serialReceive(CM_serialInterface* serial){
    char* message;
    message = serial->ESPRead();

    printf("Received message: %s \n", message);

    return 0;
}

void CM_serialSendStatus(CM_serialInterface* serial, int32_t x, int32_t y){
    char message[50];
    
    snprintf(message, sizeof(message), "Iteration: %d\n", iterations);
    printf(message);

    iterations++;

    serial->ESPWrite(message);

    return;
}

void CM_transmitStatus(int32_t x, int32_t y, int32_t a, int32_t b) {
    // Little-endian ID for status
    int32_t id = little_endian(ID_STATUS);

    uint32_t dataSize = 16;

    // Allocate buffer space (8 bytes for header + dataSize for actual data)
    uchar* sendBuffer = static_cast<uchar*>(malloc(8 + dataSize));  // 8 bytes for header and dataSize bytes for data
    if (!sendBuffer) {
        std::cerr << "Memory allocation failed." << std::endl;
        return;
    }

    dataSize = little_endian(dataSize);
    x = little_endian(x);
    y = little_endian(y);
    a = little_endian(a);
    b = little_endian(b);

    // Copy ID and dataSize into the sendBuffer (Little-endian order)
    memcpy(sendBuffer, &id, 4);  // Copy ID
    memcpy(sendBuffer + 4, &dataSize, 4);  // Copy dataSize
    memcpy(sendBuffer + 8, &x, 4);
    memcpy(sendBuffer + 12, &y, 4);
    memcpy(sendBuffer + 16, &a, 4);
    memcpy(sendBuffer + 20, &b, 4);

    // Create a UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        free(sendBuffer);
        return;
    }

    // Configure the server address
    struct sockaddr_in serverAddress;
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PC_PORT);
    inet_pton(AF_INET, PC_IP, &serverAddress.sin_addr);

    // Send the data in chunks
    size_t totalSize = 8 + dataSize;  // Header (8 bytes) + data size
    uchar* dataPtr = sendBuffer;

    for (size_t i = 0; i < totalSize; i += MAX_PACKET_SIZE) {
        size_t chunkSize = (i + MAX_PACKET_SIZE < totalSize) ? MAX_PACKET_SIZE : (totalSize - i);

        // Send data chunk using sendto
        if (sendto(sockfd, reinterpret_cast<const char*>(dataPtr + i), chunkSize, 0, 
                (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
            perror("Failed to send data chunk");
            break;
        }
    }

    // Clean up - Ensure sendBuffer is only freed once and is valid
    if (sendBuffer != nullptr) {
        free(sendBuffer);
    }

    close(sockfd);
}



void CM_transmitFrame(cv::Mat frame, int frameId) {

    // Serialize the frame
    std::vector<uchar> encodedFrame;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};

    if (!cv::imencode(".jpg", frame, encodedFrame, params)) {
        std::cerr << "Failed to encode the frame." << std::endl;
        return;
    }

    // Prepare header and data
    uint32_t frameSize = encodedFrame.size();
    uint32_t frameId_le = little_endian(frameId);  // Ensure little-endian format
    uint32_t frameSize_le = little_endian(frameSize);

    // Allocate a buffer for the header and frame data
    uchar* sendBuffer = static_cast<uchar*>(malloc(8 + frameSize));
    if (!sendBuffer) {
        std::cerr << "Memory allocation failed." << std::endl;
        return;
    }

    // Copy the header (frame ID and size) into the buffer
    memcpy(sendBuffer, &frameId_le, 4);
    memcpy(sendBuffer + 4, &frameSize_le, 4);

    // Copy the encoded frame data into the buffer
    memcpy(sendBuffer + 8, encodedFrame.data(), frameSize);

    // Create a UDP socket
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        free(sendBuffer);
        return;
    }

    // Configure the server address
    struct sockaddr_in serverAddress;
    memset(&serverAddress, 0, sizeof(serverAddress));
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(PC_PORT);
    inet_pton(AF_INET, PC_IP, &serverAddress.sin_addr);

    // Send the data in chunks
    size_t totalSize = 8 + frameSize;  // Header (8 bytes) + frame data
    uchar* dataPtr = sendBuffer;

    for (size_t i = 0; i < totalSize; i += MAX_PACKET_SIZE) 
    {
        size_t chunkSize = (i + MAX_PACKET_SIZE < totalSize) ? MAX_PACKET_SIZE : (totalSize - i);
        
        // Cast uchar* to const char* explicitly for the sendto function
        if (sendto(sockfd, reinterpret_cast<const char*>(dataPtr + i), chunkSize, 0, 
                (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
            perror("Failed to send data chunk");
            break;
        }
    }

    // Clean up
    close(sockfd);
    free(sendBuffer);
}

int CM_initNet() {
    #ifdef _WIN32
        WSADATA wsaData;
        return WSAStartup(MAKEWORD(2, 2), &wsaData);
    #else
        return 0;
    #endif
}

void CM_cleanupNet() {
    #ifdef _WIN32
        WSACleanup();
    #endif
}


bool CM_serialInterface::ESPOpen() {

    struct sp_port **ports;

    if (sp_list_ports(&ports) != SP_OK) {
        fprintf(stderr, "Error finding serial ports\n");
        return 1;
    }

    for (int i = 0; ports[i] != NULL; i++) {
        const char *portName = sp_get_port_name(ports[i]);
        const char *description = sp_get_port_description(ports[i]);

        printf(description);

        if (strstr(description, "CP210") != NULL) {
            printf("ESP32 on port: %s\n", portName ? portName : "N/A");
            if(sp_get_port_by_name(portName, &this->ESPPort) != SP_OK){
                printf("Failed to find port by name.\n");
                return 0;
            }
        }
    }

    if (sp_open(this->ESPPort, SP_MODE_READ_WRITE) != SP_OK) {
        printf("Failed to open ESP32 in read-write mode.\n");
    } else {
        sp_set_baudrate(this->ESPPort, 115200);
        printf("ESP32 opened in read-write mode.\n");
    }

    sp_free_port_list(ports); // Free the list of ports
    
    return 0;
}

void CM_serialInterface::ESPClose(){
    sp_close(this->ESPPort);
    return;
}

bool CM_serialInterface::ESPWrite(char* message){

    // Write the message to the serial port
    int bytes_written = sp_blocking_write(this->ESPPort, message, strlen(message), this->timeout);
    if (bytes_written < 0) {
        std::cerr << "Failed to write to ESP" << std::endl;
        return 1;   // Failed to write to ESP
    }

    return 0;
}

char* CM_serialInterface::ESPRead(){
    char read_buffer[1];          // Buffer to read one character at a time
    size_t buffer_size = 256;     // Initial buffer size
    char *response = (char*)malloc(buffer_size);
    if (!response) {
        fprintf(stderr, "Memory allocation failed.\n");
        return NULL;
    }
    size_t total_read = 0;

    char termination_char = '\n'; // The character to stop reading at

    while (little_endian(read_buffer[0]) != termination_char) {
        // Read one byte at a time
        int result = sp_blocking_read(this->ESPPort, read_buffer, 1, this->timeout);
        if (result < 0) {
            fprintf(stderr, "Error reading from serial port.\n");
            free(response);
            return NULL;
        } else if (result == 0) {
            // If no data in timeout return
            response[total_read] = 'X';
            break;
        }

        // Append the byte to the response buffer
        response[total_read] = little_endian(read_buffer[0]);
        total_read++;
    }

    // Null-terminate the response
    response[total_read] = '\0';

    return response;
}



//==============================================================================
// End of File : Software/src/Communication.cpp

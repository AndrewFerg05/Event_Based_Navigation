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
#define IDLE    2


#define TEST_IMAGE  "../example.jpg"
#define TEST_RUN_TIME 30

#define MAX_PACKET_SIZE 65507            // Max packet in bytes for UDP
#define PC_IP           "10.12.7.29" // Change to base station IP (SARK's laptop)
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
    DataAcquisition* dataAcquistion_,
    std::shared_ptr<CommunicationManager> comms,
    CM_serialInterface* serial) {

    auto start_time = std::chrono::steady_clock::now();   

    std::uint8_t commandReceived = 100; //Get this from external serial source
    std::uint8_t command = IDLE; //Get this from external source

    bool state_change_called = false; //Used to only set the atomics once
	
    // Frames for transmitting
    ImageData frameCamera;
    TrackedFrames frameEvents;
    cv::Mat frameTest = cv::imread(TEST_IMAGE);
    if (frameTest.empty()) {
        LOG(ERROR) << "CM: Failed to load test image. ";
        command = STOP;
    }

    cv::Mat frame;

    // Found pose
    OtherData pose;

    int bufferSize = 0;
    std::optional<int> last_output;
    while (true) {

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

        // Thread control
        if (command == RUN) {
            if(state_change_called){
                LOG(INFO) << "CM: Changed to run state ";
                dataAcquistion_->start();
                frontend_state = ThreadState::Run;
                backend_state = ThreadState::Run;
                state_change_called = false;
            }

            // ESP Receive code mode
            // commandReceived = CM_serialReceive(serial);
            // if (commandReceived != 42){     // If not no response
            //     command = commandReceived;
            // }
            if (elapsed > TEST_RUN_TIME)
            {
                state_change_called = true;
                command = STOP;
            }

            // Base Station Communication
            //      Get frames from DA and transmit on UDP
            frameCamera = comms->getFrameCamera();
            if (frameCamera.empty()) {
                // No camera frame ready
            }
            else {
                LOG(INFO) << "CM: Frame data made it to CM, transmitting...";
                CM_transmitFrame(frameCamera, 0);
            }

            frameEvents = comms->getFrameEvents();
            if (frameEvents.empty()) {
                // No event frame ready
            }
            else {
                CM_transmitFrame(frameEvents, 1);
            }

            pose = comms->getPose();
            if (pose == 0) {
                // No camera frame ready
            }
            else {
                //Transmit position estimate from BE
                CM_transmitStatus(iterations, iterations, 0, pose, iterations, iterations);

                //Send to ESP32 position estimate from BE
                CM_serialSendStatus(serial, pose, iterations);
            }
            //Transmit position estimate from BE
            CM_transmitStatus(iterations, iterations, 0, pose, iterations, iterations);

            //Send to ESP32 position estimate from BE
            CM_serialSendStatus(serial, pose, iterations);

            
        } else if (command == STOP) {
            LOG(INFO) << "CM: STOP Looping";

            // Stop Condition
            if(state_change_called){
                LOG(INFO) << "CM: Changed to stop state ";
                dataAcquistion_->stop();
                frontend_state = ThreadState::Stop;
                backend_state = ThreadState::Stop;
                state_change_called = false;
            }

            data_DA->stop_queue();  //Wake FE if waiting on data
            break;

        } else if (command == IDLE) {
            LOG(INFO) << "CM: IDLE Looping";

            // Pause Condition
           if(state_change_called){
            LOG(INFO) << "CM: Changed to idle state ";
                data_sync_state = ThreadState::Idle;
                frontend_state = ThreadState::Idle;
                backend_state = ThreadState::Idle;
                state_change_called = false;
            }

            // Wait for run command from ESP
            // commandReceived = CM_serialReceive(serial);
            // if (commandReceived != 42){     // If not no response
            //     command = commandReceived;
            // }

            sleep_ms(100);
            
            //(FOR TESTING WITHOUT ESP)
            if (elapsed > 1)
            {
                LOG(INFO) << "CM: Transitioning to RUN";
                state_change_called = true;
                command = RUN;
            }
        }
    }
}

std::uint8_t CM_serialReceive(CM_serialInterface* serial){
    char* message;

    if (serial->ESPCheckOpen() == 1)
    {
        while (serial->ESPCheckBuffer() > 0) {
        message = serial->ESPRead();
        printf("Received message: %s \n", message);
        }
    }

    return 42;
}

void CM_serialSendStatus(CM_serialInterface* serial, int32_t x, int32_t y){

    iterations++;
    LOG(INFO) << "CM : Iteration " << std::to_string(iterations);

    if (serial->ESPCheckOpen() == 1)
    {
        char message[50];
        
        snprintf(message, sizeof(message), "Iteration: %d\n", iterations);
        // printf(message);

        serial->ESPWrite(message);
    }

    return;
}

void CM_transmitStatus(int32_t x, int32_t y, int32_t z, int32_t yaw, int32_t pitch, int32_t roll) {
    int32_t id = little_endian(ID_STATUS);
    uint32_t dataSize = 24;

    // Allocate buffer space (8 bytes for header + dataSize for actual data)
    uchar* sendBuffer = static_cast<uchar*>(malloc(8 + dataSize));  // 8 bytes for header and dataSize bytes for data
    if (!sendBuffer) {
        std::cerr << "Memory allocation failed." << std::endl;
        return;
    }

    dataSize = little_endian(dataSize);
    x = little_endian(x);
    y = little_endian(y);
    z = little_endian(z);
    yaw = little_endian(yaw);
    pitch = little_endian(pitch);
    roll = little_endian(roll);

    // Copy ID and dataSize into the sendBuffer (Little-endian order)
    memcpy(sendBuffer, &id, 4);
    memcpy(sendBuffer + 4, &dataSize, 4);
    memcpy(sendBuffer + 8, &x, 4);
    memcpy(sendBuffer + 12, &y, 4);
    memcpy(sendBuffer + 16, &z, 4);
    memcpy(sendBuffer + 20, &yaw, 4);
    memcpy(sendBuffer + 24, &pitch, 4);
    memcpy(sendBuffer + 28, &roll, 4);

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
        LOG(ERROR) << "CM: Failed to encode frame for UDP";
        return;
    }

    // Prepare header and data
    uint32_t frameSize = encodedFrame.size();
    uint32_t frameId_le = little_endian(frameId);  // Ensure little-endian format
    uint32_t frameSize_le = little_endian(frameSize);

    // Allocate a buffer for the header and frame data
    uchar* sendBuffer = static_cast<uchar*>(malloc(8 + frameSize));
    if (!sendBuffer) {
        LOG(ERROR) << "CM: Failed to allocate memory for UDP (frame)";
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
        LOG(ERROR) << "CM: Failed to initialise UDP socket";
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
            LOG(ERROR) << "CM: Failed to transmit UDP data chunk";
            break;
        }
    }

    LOG(INFO) << "CM: Sent frame";

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
        LOG(ERROR) << "CM: Failed to open serial ports";
        return 1;
    }

    for (int i = 0; ports[i] != NULL; i++) {
        const char *portName = sp_get_port_name(ports[i]);
        const char *description = sp_get_port_description(ports[i]);

        //printf(description);

        if (strstr(description, "CP210") != NULL) {
            //printf("ESP32 on port: %s\n", portName ? portName : "N/A");
            if(sp_get_port_by_name(portName, &this->ESPPort) != SP_OK){
                LOG(ERROR) << "CM: Failed to find the ESP32 device ";
                return 1;
            }
        }
    }

    if (sp_open(this->ESPPort, SP_MODE_READ_WRITE) != SP_OK) {
        LOG(ERROR) << "CM: Failed to open ESP32 in read/write";
        return 1;
    } else {
        sp_set_baudrate(this->ESPPort, 115200);
        this->open = 1;
    }

    sp_free_port_list(ports); // Free the list of ports
    
    return 0;
}

void CM_serialInterface::ESPClose() {
    if (this->open == 1){
        sp_close(this->ESPPort);
        this->open = 0;
    }
    return;
}

bool CM_serialInterface::ESPWrite(char* message) {
    if (this->open == 1){
        // Write the message to the serial port
        int bytes_written = sp_blocking_write(this->ESPPort, message, strlen(message), this->timeout);
        if (bytes_written < 0) {
            LOG(ERROR) << "Failed to write to ESP";
            return 1;   // Failed to write to ESP
        }
    }

    return 0;
}

char* CM_serialInterface::ESPRead() {

    if (this->open == 1){
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
    char *response;
    response[0] = '\0';
    return response;
}

cv::Mat CM_formatCameraFrame(ImageData image) {

    int type;
    if (image.encoding == "mono8") {
        type = CV_8UC1;  // 8-bit single-channel (grayscale)
    } else if (image.encoding == "bgr8") {
        type = CV_8UC3;  // 8-bit 3-channel (BGR)
    } else if (image.encoding == "rgb8") {
        type = CV_8UC3;  // 8-bit 3-channel (RGB)
    } else if (image.encoding == "mono16") {
        type = CV_16UC1; // 16-bit single-channel (grayscale)
    } else {
        LOG(ERROR) << "CM: Unsupported encoding of camera frame: " << image.encoding;
        cv::Mat frame;
        return frame;
    }

    cv::Mat frame(image.height, image.width, type, const_cast<uint8_t*>(image.data.data()));

    // OpenCV uses BGR, if the input encoding is RGB, we must convert it
    if (image.encoding == "rgb8") {
        //cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    return frame;
}

cv::Mat CM_formatEventFrame(TrackedFrames image) {

    int type;
    if (image.encoding == "mono8") {
        type = CV_8UC1;  // 8-bit single-channel (grayscale)
    } else if (image.encoding == "bgr8") {
        type = CV_8UC3;  // 8-bit 3-channel (BGR)
    } else if (image.encoding == "rgb8") {
        type = CV_8UC3;  // 8-bit 3-channel (RGB)
    } else if (image.encoding == "mono16") {
        type = CV_16UC1; // 16-bit single-channel (grayscale)
    } else {
        LOG(ERROR) << "CM: Unsupported encoding of camera frame: " << image.encoding;
        cv::Mat frame;
        return frame;
    }

    cv::Mat frame(image.height, image.width, type, const_cast<uint8_t*>(image.data.data()));

    // OpenCV uses BGR, if the input encoding is RGB, we must convert it
    if (image.encoding == "rgb8") {
        //cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }

    return frame;
}

//==============================================================================
// End of File : Software/src/Communication.cpp

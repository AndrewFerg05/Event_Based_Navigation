/*
Filename    : Software/src/FrontEnd.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Front End code for determining camera pose
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
#include "FrontEnd.hpp"

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------

//==============================================================================
// Functions
//------------------------------------------------------------------------------



// Define the event frame size for the DAVIS 346 camera
const int EVENT_FRAME_WIDTH = 346;
const int EVENT_FRAME_HEIGHT = 260;

// Accumulator image to store events (floating point for smooth decay)
cv::Mat event_frame = cv::Mat::zeros(EVENT_FRAME_HEIGHT, EVENT_FRAME_WIDTH, CV_32FC3);

// Exponential decay constant (adjust for faster/slower fading)
const float DECAY_CONSTANT = 200.0f;  // Higher values mean faster fading

// Time tracking for correct decay computation
auto last_update_time = std::chrono::high_resolution_clock::now();

// Function to accumulate and display events
void displayStampedEvents(const StampedEventArray& stamped_events) {
    // Compute elapsed time since the last update
    auto current_time = std::chrono::high_resolution_clock::now();
    float elapsed_time = std::chrono::duration<float>(current_time - last_update_time).count();
    last_update_time = current_time;

    // Compute exponential decay factor: e^(-λ * Δt)
    float decay_factor = std::exp(-DECAY_CONSTANT * elapsed_time);

    // Apply **true exponential decay** based on elapsed time
    event_frame *= decay_factor;

    // Extract timestamp and event array pointer
    int64_t timestamp = stamped_events.first;
    EventArrayPtr eventArrayPtr = stamped_events.second;

    if (!eventArrayPtr || eventArrayPtr->empty()) {
        std::cerr << "Warning: No events received at timestamp " << timestamp << std::endl;
        return;
    }

    // Iterate through all events and plot them on the event_frame
    for (const Event& e : *eventArrayPtr) {
        if (e.x >= EVENT_FRAME_WIDTH || e.y >= EVENT_FRAME_HEIGHT) {
            continue;  // Ignore out-of-bounds events
        }

        // Set pixel intensity based on event polarity
        if (e.polarity) {
            event_frame.at<cv::Vec3f>(e.y, e.x) = cv::Vec3f(1.0f, 1.0f, 1.0f); // ON events → White
        } else {
            event_frame.at<cv::Vec3f>(e.y, e.x) = cv::Vec3f(0.0f, 0.0f, 1.0f); // OFF events → Blue
        }
    }

    // Convert to 8-bit image for display
    cv::Mat display_frame;
    event_frame.convertTo(display_frame, CV_8UC3, 255.0);
    cv::GaussianBlur(display_frame, display_frame, cv::Size(3,3), 1.0);
    // Display the accumulated event frame
    cv::imshow("Event Camera Stream", display_frame);

    // Short delay for smooth video playback
    cv::waitKey(1);
}


// Function to display images in a continuous video stream
void displayStampedImage(const StampedImage& stamped_image) {
    // Extract timestamp and image pointer
    int64_t timestamp = stamped_image.first;
    ImagePtr imagePtr = stamped_image.second;

    if (!imagePtr) {
        std::cerr << "Error: Image pointer is null!" << std::endl;
        return;
    }

    // Extract image properties
    int width = imagePtr->width;
    int height = imagePtr->height;
    std::string encoding = imagePtr->encoding;
    std::vector<uint8_t> data = imagePtr->data;

    if (data.empty()) {
        std::cerr << "Error: Image data is empty!" << std::endl;
        return;
    }

    // Convert the data vector to cv::Mat
    cv::Mat img;
    if (encoding == "mono8") {
        img = cv::Mat(height, width, CV_8UC1, data.data());
    } else if (encoding == "rgb8") {
        cv::Mat temp = cv::Mat(height, width, CV_8UC3, data.data());
        cv::cvtColor(temp, img, cv::COLOR_RGB2BGR); // Convert RGB to OpenCV's BGR
    } else {
        std::cerr << "Error: Unsupported encoding format: " << encoding << std::endl;
        return;
    }

    // Display the image in an OpenCV window
    cv::imshow("Stamped Image Stream", img);
    
    // Short delay to allow OpenCV to refresh the display
    cv::waitKey(1);  // 1ms delay, ensures smooth video playback
}



FrontEnd::FrontEnd(std::shared_ptr<CommunicationManager> comms)
    : comms_interface_(comms)
    {}


void FrontEnd::start()
{

}

void FrontEnd::idle()
{
    
}

void FrontEnd::stop()
{
    
}

void FrontEnd::addData(
    const StampedImage& stamped_image,
    const StampedEventArray& stamped_events,
    const ImuStamps& imu_stamps,
    const ImuAccGyrContainer& imu_accgyr,
    const bool& no_motion_prior)
{
    // Test functions will only run in main thread
    // displayStampedImage(stamped_image);
    // displayStampedEvents(stamped_events);
}

void FrontEnd::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr)
{

}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp

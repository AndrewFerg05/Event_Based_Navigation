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

// Frames for event data and APS data
cv::Mat event_frame = cv::Mat::zeros(EVENT_FRAME_HEIGHT, EVENT_FRAME_WIDTH, CV_32FC3);
cv::Mat aps_frame;  // Will store the latest APS frame

// Function to display events from each incoming packet separately
void displayStampedEvents(const StampedEventArray& stamped_events) {
    // Clear the event frame at the start of each function call
    event_frame = cv::Mat::zeros(EVENT_FRAME_HEIGHT, EVENT_FRAME_WIDTH, CV_32FC3);

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

    // Convert event frame to 8-bit (CV_8UC3)
    cv::Mat event_frame_8bit;
    event_frame.convertTo(event_frame_8bit, CV_8UC3, 255.0); // Scale float to 8-bit

    // Define a kernel for morphology operations
    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // // Apply morphological opening (removes small noise)
    // cv::morphologyEx(event_frame_8bit, event_frame_8bit, cv::MORPH_OPEN, kernel);

    // // Apply morphological closing (fills small gaps)
    // cv::morphologyEx(event_frame_8bit, event_frame_8bit, cv::MORPH_CLOSE, kernel);

    // Display the cleaned event frame
    cv::imshow("Processed Event Frame", event_frame_8bit);

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
    if (encoding == "mono8") {
        aps_frame = cv::Mat(height, width, CV_8UC1, data.data());
        cv::cvtColor(aps_frame, aps_frame, cv::COLOR_GRAY2BGR);  // Convert grayscale to 3-channel BGR
    } else if (encoding == "rgb8") {
        cv::Mat temp = cv::Mat(height, width, CV_8UC3, data.data());
        cv::cvtColor(temp, aps_frame, cv::COLOR_RGB2BGR); // Convert RGB to OpenCV's BGR
    } else {
        std::cerr << "Error: Unsupported encoding format: " << encoding << std::endl;
        return;
    }

    // Display the APS image
    cv::imshow("Stamped Image Stream", aps_frame);
    
    // Short delay to allow OpenCV to refresh the display
    cv::waitKey(1);  // 1ms delay, ensures smooth video playback
}

void displayCombinedFrame() {
    if (aps_frame.empty() || event_frame.empty()) {
        std::cerr << "Warning: One or both frames are empty!" << std::endl;
        return;
    }

    // Ensure both images are the same size
    if (aps_frame.size() != event_frame.size()) {
        cv::resize(aps_frame, aps_frame, event_frame.size());
    }

    // Convert event_frame to 8-bit (CV_8UC3) for blending
    cv::Mat event_frame_8bit;
    event_frame.convertTo(event_frame_8bit, CV_8UC3, 255.0); // Scale float to 8-bit

    // Blend APS frame with event frame
    cv::Mat combined_frame;
    cv::addWeighted(aps_frame, 0.7, event_frame_8bit, 0.5, 0, combined_frame);

    // Display the combined frame
    cv::imshow("Combined Event + APS Frame", combined_frame);

    // Short delay for smooth video playback
    cv::waitKey(1);
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
    displayStampedImage(stamped_image);
    displayStampedEvents(stamped_events);
    displayCombinedFrame();
}

void FrontEnd::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr)
{

}
//==============================================================================
// End of File : Software/src/DataAcquisition.cpp

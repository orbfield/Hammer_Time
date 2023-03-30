#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <vector>
#include "C:\Dev\Iibaries\rtmidi-5.0.0\RtMidi.h"
#include "HammerSound.h"
#include <thread>
#include <chrono>

// Raw midi function
int time_to_velocity(double elapsed_time_ms) {
    double min_time = 10.0;
    double max_time = 75.0;
    int max_velocity = 127;
    int min_velocity = 1;

    if (elapsed_time_ms <= min_time) {
        return max_velocity;
    }
    else if (elapsed_time_ms >= max_time) {
        return min_velocity;
    }
    else {
        double velocity = max_velocity - (max_velocity - min_velocity) * (elapsed_time_ms - min_time) / (max_time - min_time);
        return static_cast<int>(velocity);
    }
}

// Midi velocity curve function 
int ConvertMidiValue(int value, float deviation) {
    if (deviation < -100.0f || deviation > 100.0f) {
        throw std::invalid_argument("Value must be between -100 and 100");
    }
    float minMidiValue = 0.0f;
    float maxMidiValue = 127.0f;
    float midMidiValue = 63.5f;

    // Control point for the quadratic bezier curve - range: 0 (min) to 63.5 (max)
    float controlPointX = midMidiValue + ((deviation / 100.0f) * midMidiValue);

    // Get the percent position of the incoming value in relation to the max
    float t = static_cast<float>(value) / maxMidiValue;

    // The quadratic bezier curve formula
    int delta = static_cast<int>(std::round((2.0f * (1.0f - t) * t * controlPointX) + (t * t * maxMidiValue)));

    return (value - delta) + value;
}

// Ready Synth
void run_hammer_sound(std::shared_ptr<HammerSound> hammerSound) {
    hammerSound->run();
}

int main()
{    
    double fps = 0;
    double last_tick = 0;
 
    // Create a shared HammerSound object
    auto hammerSound = std::make_shared<HammerSound>();

    // Run synthesizer module on a separate thread
    std::thread hammer_sound_thread(run_hammer_sound, hammerSound);


    cv::namedWindow("Hammer Time", cv::WINDOW_NORMAL);

    cv::VideoCapture cap(cv::CAP_DSHOW);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 620);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 120);
    cap.set(cv::CAP_PROP_EXPOSURE, -7);

    if (!cap.isOpened()) {
        std::cout << "Error opening video stream" << std::endl;
        return -1;
    }

    // Define the grid parameters and labels
    const int num_rows = 1;
    const int num_cols = 20;
    const float box_width = 32;
    const int box_width_int = static_cast<int>(box_width);
    const int box_height = 480;

    std::vector<std::string> col_labels = { "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A" };

    // Define switches
    std::vector<bool> switch1_state(num_cols, false);
    std::vector<bool> switch2_state(num_cols, false);
    bool prev_switch1_state[num_cols] = { false };
    bool prev_switch2_state[num_cols] = { false };
    //const double switch_distance = 0.0127;

    // Define switch timer 
    std::vector<cv::TickMeter> start_switch_timer(num_cols);

    // Loop over the video frames
    while (true) {
        // Capture a new frame from the webcam
        cv::Mat frame;
        cap.read(frame);
        if (frame.empty()) {
            break;
        }

        // Draw a column for each hammer
        for (int i = 0; i < num_rows; i++) {
            int y = i * box_height;
            for (int j = 0; j < num_cols; j++) {
                int x = j * box_width;
                cv::Rect box(x, y, box_width, box_height);
                cv::rectangle(frame, box, cv::Scalar(0, 128, 0), 1);
            }
        }

        // Draw the column labels
        cv::Point text_origin(0, frame.rows - box_height / 2);
        for (int j = 0; j < num_cols; j++) {
            int x = j * box_width + box_width / 2;
            std::string label = col_labels[j];
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.3, 1, nullptr);
            cv::Point label_origin = text_origin + cv::Point(x - text_size.width / 2, -8);
            cv::putText(frame, label, label_origin, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 128, 0), 1);
        }

        // Plot switch 1 and switch 2 for each note
        for (int j = 0; j < num_cols; j++) {
            int x = j * box_width;
            cv::Rect roi1(x, frame.rows * 0.49, box_width, 4);
            cv::Rect roi2(x, frame.rows * 0.41, box_width, 4);
            cv::Mat roi_image1 = frame(roi1);
            cv::Mat roi_image2 = frame(roi2);

            // Image processing
            cv::cvtColor(roi_image1, roi_image1, cv::COLOR_BGR2GRAY);
            cv::cvtColor(roi_image2, roi_image2, cv::COLOR_BGR2GRAY);

            cv::Scalar roi_mean1 = cv::mean(roi_image1);
            cv::Scalar roi_mean2 = cv::mean(roi_image2);

            cv::Mat roi_binary1, roi_binary2;
            cv::threshold(roi_image1, roi_binary1, roi_mean1.val[0], 255, cv::THRESH_BINARY);
            cv::threshold(roi_image2, roi_binary2, roi_mean2.val[0], 255, cv::THRESH_BINARY);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4, 4));
            cv::erode(roi_binary1, roi_binary1, kernel, cv::Point(-1, -1), 1);
            cv::dilate(roi_binary1, roi_binary1, kernel, cv::Point(-1, -1), 1);
            cv::erode(roi_binary2, roi_binary2, kernel, cv::Point(-1, -1), 1);
            cv::dilate(roi_binary2, roi_binary2, kernel, cv::Point(-1, -1), 1);

            // Count white pixels
            cv::Mat roi_count1, roi_count2;
            int count1 = cv::countNonZero(roi_binary1);
            int count2 = cv::countNonZero(roi_binary2);

            // Check if switch 1 is on or off 
            bool switch1_on = false;
            if (count1 > 50) {
                switch1_on = true;
            }
            if (switch1_on && !prev_switch1_state[j]) {
                // Switch 1 turned on          
                start_switch_timer[j].reset();
                start_switch_timer[j].start();
            }
            if (!switch1_on && prev_switch1_state[j]) {
                // Switch 1 turned off: send midi
                int midi_pitch = 50+ j;
                hammerSound->sendNoteOff(midi_pitch);
            }

            // Check if switch 2 is on or off 
            bool switch2_on = false;
            if (count2 > 60) {
                switch2_on = true;
            }
            if (switch2_on && !prev_switch2_state[j]) {
                // Switch 2 turned on: calculate velocity send midi                
                start_switch_timer[j].stop();
                int64 switch1_ticks = start_switch_timer[j].getTimeTicks();
                double switch1_elapsed_time = static_cast<double>(switch1_ticks) / cv::getTickFrequency();
                /*std::cout << "~ " << j << ": " << switch1_elapsed_time * 1000 << " ms\n"; */

                // Convert elapsed time to MIDI velocity
                int raw_velocity = time_to_velocity(switch1_elapsed_time * 1000);
                double deviation = 50;  // Change from 100 (lower velocities) to -100 (higher velocities) 
                int midi_velocity = ConvertMidiValue(raw_velocity, deviation);

                // Send MIDI message
                int midi_pitch = 50+ j;
                hammerSound->sendNoteOn(midi_pitch, midi_velocity);        
            }
            if (!switch2_on && prev_switch2_state[j]) {
                // Switch 2 turned off
                int midi_pitch = 50+ j;
            }

            prev_switch1_state[j] = switch1_on;
            prev_switch2_state[j] = switch2_on;

            // Draw switch boxes
            cv::Scalar color1(0, 150, 0);
            cv::Scalar color2(0, 150, 0);
            cv::rectangle(frame, roi1, color1, 1);
            cv::rectangle(frame, roi2, color2, 1);
            /*
            // Display the first 12 pairs of switches
            if (j < 12) {
                cv::Mat roi_binary_comb;
                cv::vconcat(roi_binary2, roi_binary1, roi_binary_comb);
                std::string switch_names = col_labels[j] + " switches";
                cv::namedWindow(switch_names, cv::WINDOW_NORMAL);
                cv::imshow(switch_names, roi_binary_comb);
            }*/
        }

        // Create a black mask at bottom of screen
        cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        int box_height = frame.rows / 4; // 25% of the height
        int box_width = frame.cols;
        cv::Rect roi(0, frame.rows - box_height, box_width, box_height);
        mask(roi) = cv::Scalar(255);

        cv::Mat black_image = cv::Mat::zeros(frame.size(), frame.type());
        black_image.setTo(cv::Scalar(0, 0, 0));
        black_image.copyTo(frame, mask);

        // Calculate FPS and display
        double tick = cv::getTickCount();
        double delta = (tick - last_tick) / cv::getTickFrequency();
        fps = 1.0 / delta;
        last_tick = tick;
        std::stringstream ss;
        ss << "fps " << std::fixed << std::setprecision(1) << fps;
        cv::putText(frame, ss.str(), cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 200, 0), 1);

        // Display the current frame
        cv::imshow("HammerTime", frame);

        char c = cv::waitKey(1);
        if (c == 27) {
            // Exit if the user presses the ESC key
            break;
        }
    }
    hammer_sound_thread.join();
    return 0;
}

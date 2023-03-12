#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <iostream>
#include <vector>
#include "C:\Dev\Iibaries\rtmidi-5.0.0\RtMidi.h"


int main()
{
    // FPS calculation
    double fps = 0;
    double last_tick = 0;

    RtMidiOut midi_out;
    int num_ports = midi_out.getPortCount();
    std::string port_name;
    for (int i = 0; i < num_ports; i++) {
        try {
            port_name = midi_out.getPortName(i);
            std::cout << "MIDI port #" << i << ": " << port_name << std::endl;
        }
        catch (RtMidiError& error) {
            error.printMessage();
        }
    }

    RtMidiOut midiOut;
    midiOut.openPort(1);

    // Test midi
    std::vector<unsigned char> messageOn(3);
    messageOn[0] = 0x90; // Note On message
    messageOn[1] = 60;   // pitch = middle C
    messageOn[2] = 100;  // velocity
    midiOut.sendMessage(&messageOn);

    // Haha
    cv::waitKey(1000);

    std::vector<unsigned char> messageOff(3);
    messageOff[0] = 0x80; // Note Off message
    messageOff[1] = 60;   // pitch = middle C
    messageOff[2] = 0;    // velocity
    midiOut.sendMessage(&messageOff);

    // Create webcam feed
    cv::namedWindow("Hammer Time", cv::WINDOW_NORMAL);

    cv::VideoCapture cap(cv::CAP_ANY);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 60);
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

    std::vector<std::string> col_labels = {"D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A"};
    std::vector<std::string> row_labels(num_rows);
    for (int i = 0; i < num_rows; i++) {
        row_labels[i] = std::to_string(num_rows - i);
    }

    // Define switches
    std::vector<bool> switch1_state(num_cols, false);
    std::vector<bool> switch2_state(num_cols, false);
    bool prev_switch1_state[num_cols] = { false };
    bool prev_switch2_state[num_cols] = { false };

    //define vector for switch timer
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
            cv::Point label_origin = text_origin + cv::Point(x - text_size.width / 2, -5);
            cv::putText(frame, label, label_origin, cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 128, 0), 1);
        }

        // Plot switch 1 and switch 2 for each note
        for (int j = 0; j < num_cols; j++) {
            int x = j * box_width;
            cv::Rect roi1(x, frame.rows * 0.50, box_width, 5);
            cv::Rect roi2(x, frame.rows * 0.43, box_width, 5);
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

            // Check if switches 1 is on or off 
            bool switch1_on = false;
            if (count1 > 70) {
                switch1_on = true;
            }
            if (switch1_on && !prev_switch1_state[j]) {                               
                // Switch 1 turned on          
                start_switch_timer[j].reset();
                start_switch_timer[j].start();
            }
            if (!switch1_on && prev_switch1_state[j]) {
                // Switch 1 turned off: send midi
                int midi_pitch = 62 + j;
                std::vector<unsigned char> messageOff(3);
                messageOff[0] = 0x80;
                messageOff[1] = midi_pitch;
                messageOff[2] = 0;
                midiOut.sendMessage(&messageOff);
            }

            // Check if switch 2 is on or off 
            bool switch2_on = false;
            if (count2 > 80) {
                switch2_on = true;
            }
            if (switch2_on && !prev_switch2_state[j]) {
                // Switch 2 turned on: send midi
                int midi_pitch = 62 + j;
                std::vector<unsigned char> messageOn(3);
                messageOn[0] = 0x90;
                messageOn[1] = midi_pitch;
                messageOn[2] = 100;
                midiOut.sendMessage(&messageOn);

                // Finish switch timer
                start_switch_timer[j].stop();
                int64 switch1_ticks = start_switch_timer[j].getTimeTicks();
                double switch1_elapsed_time = static_cast<double>(switch1_ticks) / cv::getTickFrequency();
                std::cout << "~ " << j << ": " << switch1_elapsed_time * 1000 << " ms\n";
            }
            if (!switch2_on && prev_switch2_state[j]) {
                // Switch 2 turned off
                int midi_pitch = 62 + j;
            }

            prev_switch1_state[j] = switch1_on;
            prev_switch2_state[j] = switch2_on;

            // Draw switch boxes
            cv::Scalar color1(0, 150, 0);
            cv::Scalar color2(0, 150, 0);
            cv::rectangle(frame, roi1, color1, 1);
            cv::rectangle(frame, roi2, color2, 1);

            // Display the first 12 pairs of switches
            if (j < 12) {
                cv::Mat roi_binary_comb;
                cv::vconcat(roi_binary2, roi_binary1, roi_binary_comb);
                std::string switch_names = col_labels[j] + " switches";
                cv::namedWindow(switch_names, cv::WINDOW_NORMAL);
                cv::imshow(switch_names, roi_binary_comb);
            
                cv::namedWindow(switch_names, cv::WINDOW_NORMAL);
                cv::imshow(switch_names, roi_binary_comb);
            }            
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
        cv::imshow("Hammer Time", frame);

        char c = cv::waitKey(1);
        if (c == 27) {
            // Exit if the user presses the ESC key
            break;
        }
    }
    return 0;
}
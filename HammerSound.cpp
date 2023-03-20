#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "HammerSound.h"
#include <RtAudio.h>
#include <C:/Dev/Iibaries/rtmidi-5.0.0/RtMidi.h>
#include <fstream>
#include <atomic>

HammerSound::HammerSound() : isPlaying(false), currentFrequency(0.0), amplitude(0.0) {}
HammerSound::~HammerSound() {}

int audioCallback(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status, void* userData) {
    HammerSound* hammerSound = static_cast<HammerSound*>(userData);
    return hammerSound->processAudioOutput(outputBuffer, inputBuffer, nBufferFrames, streamTime, status);
}

void handleError(const char* errorText, void* userData) {
    std::cerr << "RtAudio error: " << errorText << std::endl;
}

void HammerSound::sendTestBeep() {
    int testMidiPitch = 69; // A4 (440 Hz)
    int testMidiVelocity = 60;
    int testBeepDuration = 1000;

    sendNoteOn(testMidiPitch, testMidiVelocity);
    std::this_thread::sleep_for(std::chrono::milliseconds(testBeepDuration));
    sendNoteOff(testMidiPitch);
}

void HammerSound::run() {


    // Initialize audio output
    try {
        auto audioOut = std::make_unique<RtAudio>();

        if (audioOut->getDeviceCount() < 1) {
            std::cout << "No audio devices found!" << std::endl;
            return;
        }
        RtAudio::StreamParameters parameters;
        parameters.deviceId = audioOut->getDefaultOutputDevice();
        parameters.nChannels = 2;
        parameters.firstChannel = 0;

        unsigned int sampleRate = 44100;
        unsigned int bufferFrames = 256;

        RtAudio::DeviceInfo deviceInfo = audioOut->getDeviceInfo(parameters.deviceId);
        std::cout << "\033[32mDefault Output Device:\033[0m " << deviceInfo.name << std::endl;

        audioOut->openStream(&parameters, nullptr, RTAUDIO_FLOAT64,
            sampleRate, &bufferFrames, &audioCallback, this);
        audioOut->startStream();

        sendTestBeep();
    }
    catch (...) {
        std::cerr << "Unknown error occurred while initializing RtAudio." << std::endl;
        return;
    }
}
void HammerSound::sendNoteOn(int midi_pitch, int midi_velocity) {
    isPlaying.store(true);

    // Update current frequency
    currentFrequency.store(146.83 * pow(2.0, (midi_pitch - 50) / 12.0));

    // Set amplitude based on MIDI velocity
    amplitude.store(midi_velocity / 127.0);

    // Debug print statement
    std::cout << "Note ON: Midi pitch = " << midi_pitch << ", Midi velocity = " << midi_velocity << ", Frequency = " << currentFrequency.load() << " Hz\n";
}

void HammerSound::sendNoteOff(int midi_pitch) {
    isPlaying.store(false);

    // Reset current frequency
    currentFrequency.store(0.0);

    // Debug print statement
    std::cout << "Note OFF: Midi pitch = " << midi_pitch << std::endl;
}


// Process audio output
int HammerSound::processAudioOutput(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status) {

    double* outBuffer = static_cast<double*>(outputBuffer);
    
    double sampleRate = 44100.0;
    double phaseIncrement = 2.0 * M_PI * currentFrequency.load() / sampleRate;
    static double phase = 0.0;

    if (isPlaying.load()) {
        for (unsigned int i = 0; i < nBufferFrames; ++i) {
            double sampleValue = amplitude.load() * (sin(phase) >= 0 ? 1.0 : -1.0);
            outBuffer[i * 2] = sampleValue; // Left channel
            outBuffer[i * 2 + 1] = sampleValue; // Right channel
            phase += phaseIncrement;
            if (phase > 2.0 * M_PI) {
                phase -= 2.0 * M_PI;
            }
        }
        // Debug print statements - write to file
        std::ofstream debugFile("debug.txt", std::ios_base::app);
        debugFile << "sampleValue: " << outBuffer[0] << ", phase: " << phase << ", phaseIncrement: " << phaseIncrement << ", currentFrequency: " << currentFrequency << std::endl;
        debugFile.close();
    }
    else {
        memset(outBuffer, 0, nBufferFrames * 2 * sizeof(double));
    }

    return 0;
}



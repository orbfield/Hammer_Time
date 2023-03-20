#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <thread>
#include "HammerSound.h"
#include <RtAudio.h>
#include <C:/Dev/Iibaries/rtmidi-5.0.0/RtMidi.h>
#include <atomic>
#include <fstream>

HammerSound::HammerSound() : isPlaying(false), currentFrequency(0.0), amplitude(0.0) {}
HammerSound::~HammerSound() {}

int audioCallback(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status, void* userData) {
    HammerSound* hammerSound = static_cast<HammerSound*>(userData);
    return hammerSound->processAudioOutput(outputBuffer, inputBuffer, nBufferFrames, streamTime, status);
}


void midiInputCallback(double deltatime, std::vector<unsigned char>* message, void* userData) {
    // Cast userData to HammerSound object
    HammerSound* hammerSound = static_cast<HammerSound*>(userData);

    unsigned int nBytes = message->size();

    if (nBytes > 0) {
        unsigned char statusByte = message->at(0);

        if (statusByte == 0x90 && nBytes == 3) { // Note On message
            int midiPitch = message->at(1);
            int midiVelocity = message->at(2);

            if (midiVelocity > 0) {
                hammerSound->sendNoteOn(midiPitch, midiVelocity);
            }
            else {
                hammerSound->sendNoteOff(midiPitch);
            }
        }
        else if (statusByte == 0x80 && nBytes == 3) { // Note Off message
            int midiPitch = message->at(1);
            hammerSound->sendNoteOff(midiPitch);
        }
    }
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
 
    // Initialize audio output and midi input
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

        try {
            auto midiIn = std::make_unique<RtMidiIn>();

            if (midiIn->getPortCount() == 0) {
                std::cout << "No MIDI input ports available!" << std::endl;
                return;
            }

            midiIn->openPort(0);
            midiIn->setCallback(&midiInputCallback, this);
            midiIn->ignoreTypes(false, false, false);
            std::cout << "MIDI input opened on port: " << midiIn->getPortName(0) << std::endl;
          
            char input;
            std::cout << "Press any key and hit enter to quit." << std::endl;
            std::cin >> input;
            midiIn->closePort();
            audioOut->closeStream();
        }
        catch (RtMidiError& error) {
            error.printMessage();
            return;
        }
        catch (...) {
            std::cerr << "Unknown error occurred while initializing RtMidi.\n";
            return;
        }
        
    }
    catch (...) {
        std::cerr << "Unknown error occurred while initializing RtAudio.\n";
        return;
    }
}

void HammerSound::sendNoteOn(int midi_pitch, int midi_velocity) {
    isPlaying.store(true);
       
    currentFrequency.store(146.83 * pow(2.0, (midi_pitch - 50) / 12.0)); 
    amplitude.store(midi_velocity / 127.0);
    
    std::cout << "Note ON: Midi pitch = " << midi_pitch << ", Midi velocity = " << midi_velocity << ", Frequency = " << currentFrequency.load() << " Hz\n";
}

void HammerSound::sendNoteOff(int midi_pitch) {
    isPlaying.store(false);
    
    currentFrequency.store(0.0);
    std::cout << "Note OFF: Midi pitch = " << midi_pitch << " \n";
}

// Process audio output
int HammerSound::processAudioOutput(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status) {

    double* outBuffer = static_cast<double*>(outputBuffer);
    
    double sampleRate = 44100.0;
    double phaseIncrement = 2.0 * M_PI * currentFrequency.load() / sampleRate;
    static double phase = 0.0;
    double carrierFrequency = currentFrequency.load();
    double modulatorFrequency = carrierFrequency * 1.5; // You can experiment with this value
    double modulatorAmplitude = 0.1; // You can experiment with this value


    if (isPlaying.load()) {
        for (unsigned int i = 0; i < nBufferFrames; ++i) {
            static double modulatorPhase = 0.0;
            double modulatorSignal = modulatorAmplitude * sin(modulatorPhase);
            double sampleValue = amplitude.load() * sin(phase + modulatorSignal);

            modulatorPhase += 2.0 * M_PI * modulatorFrequency / sampleRate;
            if (modulatorPhase > 2.0 * M_PI) {
                modulatorPhase -= 2.0 * M_PI;
            }
            outBuffer[i * 2] = sampleValue; // Left channel
            outBuffer[i * 2 + 1] = sampleValue; // Right channel
            phase += phaseIncrement;
            if (phase > 2.0 * M_PI) {
                phase -= 2.0 * M_PI;
            }
        }
    }
    else {
        memset(outBuffer, 0, nBufferFrames * 2 * sizeof(double));
    }

    return 0;
}



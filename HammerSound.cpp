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
#include <mutex>

HammerSound::HammerSound()
    : isPlaying(false),
    currentFrequency(0.0),
    amplitude(0.0),
    compressor(/*threshold=*/0.1, /*ratio=*/3.0, /*attackTime=*/0.01, /*releaseTime=*/0.2),
    adsrEnvelope(0.005, 0.05, 1, 0.05) {
    createSineWaveTable(sineWaveTable);
}

HammerSound::~HammerSound() {}

void HammerSound::createSineWaveTable(std::vector<double>& waveTable) {
    waveTable.resize(WAVE_TABLE_SIZE);

    for (unsigned int i = 0; i < WAVE_TABLE_SIZE; ++i) {
        waveTable[i] = sin(2.0 * M_PI * i / WAVE_TABLE_SIZE);
    }
}
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
    int fadeDuration = 50; // 50ms fade-in duration

    double fadeInStep = static_cast<double>(testMidiVelocity) / fadeDuration;

    // Apply fade-in
    for (int i = 0; i <= fadeDuration; ++i) {
        int velocityStep = static_cast<int>(fadeInStep * i);
        sendNoteOn(testMidiPitch, velocityStep);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(testBeepDuration - fadeDuration));

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
// Update sendNoteOn function
void HammerSound::sendNoteOn(int midi_pitch, int midi_velocity) {
    double frequency = 146.83 * pow(2.0, (midi_pitch - 50) / 12.0);
    double amplitude = midi_velocity / 127.0;

    std::unique_lock<std::mutex> lock(activeNotesMutex);
    activeNotes.push_back({ frequency, amplitude, 0.0, 0.0, true, 0.0, adsrEnvelope });
    activeNotes.back().adsrEnvelope.noteOn(); // Trigger the envelope
    lock.unlock();

    std::cout << "Note ON: Midi pitch = " << midi_pitch << ", Midi velocity = " << midi_velocity << ", Frequency = " << frequency << " Hz\n";
}

// Update sendNoteOff function
void HammerSound::sendNoteOff(int midi_pitch) {
    double frequency = 146.83 * pow(2.0, (midi_pitch - 50) / 12.0);
    for (ActiveNote& note : activeNotes) {
        if (note.frequency == frequency) {
            note.adsrEnvelope.noteOff(); // Trigger the release phase
        }
    }
    std::cout << "Note OFF: Midi pitch = " << midi_pitch << " \n";
}


int HammerSound::processAudioOutput(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status) {

    double* outBuffer = static_cast<double*>(outputBuffer);

    double sampleRate = 44100.0;
    double carrierFrequency;

    std::unique_lock<std::mutex> lock(activeNotesMutex);
    for (unsigned int i = 0; i < nBufferFrames; ++i) {
        double mixedSample = 0.0;

        for (ActiveNote& note : activeNotes) {
            carrierFrequency = note.frequency;
            unsigned int tableIndex = static_cast<unsigned int>(note.phase) % WAVE_TABLE_SIZE;

            double envelopeAmplitude = note.amplitude * note.adsrEnvelope.process(sampleRate);
            mixedSample += envelopeAmplitude * sineWaveTable[tableIndex];

            note.phase += WAVE_TABLE_SIZE * carrierFrequency / sampleRate;
            note.time += 1.0 / sampleRate;
        }

        // Apply a simple mixing method to prevent clipping
        double scaleFactor = 1.0 / std::sqrt(activeNotes.size());
        mixedSample *= scaleFactor;

        outBuffer[i * 2] = mixedSample; // Left channel
        outBuffer[i * 2 + 1] = mixedSample; // Right channel
    }
    lock.unlock();

    return 0;
}

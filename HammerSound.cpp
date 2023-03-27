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
    adsrEnvelope(/*attack*/0.002, /*decay*/0.01, /*sustain*/0.9, /*release*/0.01),
    compressor(/*threshold=*/0.05, /*ratio=*/15.0, /*attack=*/0.005, /*release=*/0.02,/*kneeWidth=*/0.2),
    delay(/*delayTime=*/0.200, /*feedback=*/0.0, /*mix=*/0.3, /*sampleRate=*/44100.0),
    masterGain(1.0/*dB*/) {
    midiOut = std::make_shared<RtMidiOut>();
    createSineWaveTable(sineWaveTable);
    createSquareWaveTable(squareWaveTable);
    createSawToothWaveTable(sawtoothWaveTable);
    createTriangleWaveTable(triangleWaveTable);
}

HammerSound::~HammerSound() {}

void HammerSound::createSineWaveTable(std::vector<double>& waveTable) {
    waveTable.resize(WAVE_TABLE_SIZE);

    for (unsigned int i = 0; i < WAVE_TABLE_SIZE; ++i) {
        waveTable[i] = sin(2.0 * M_PI * i / WAVE_TABLE_SIZE);
    }
}

void HammerSound::createSquareWaveTable(std::vector<double>& waveTable) {
    waveTable.resize(WAVE_TABLE_SIZE);

    for (unsigned int i = 0; i < WAVE_TABLE_SIZE; ++i) {
        waveTable[i] = (i < WAVE_TABLE_SIZE / 2) ? 1.0 : -1.0;
    }
}
void HammerSound::createSawToothWaveTable(std::vector<double>& waveTable) {
    waveTable.resize(WAVE_TABLE_SIZE);

    for (unsigned int i = 0; i < WAVE_TABLE_SIZE; ++i) {
        waveTable[i] = 2.0 * (i / (double)WAVE_TABLE_SIZE) - 1.0;
    }
}
void HammerSound::createTriangleWaveTable(std::vector<double>& waveTable) {
    waveTable.resize(WAVE_TABLE_SIZE);

    for (unsigned int i = 0; i < WAVE_TABLE_SIZE / 2; ++i) {
        waveTable[i] = 2.0 * (i / (double)WAVE_TABLE_SIZE);
    }
    for (unsigned int i = WAVE_TABLE_SIZE / 2; i < WAVE_TABLE_SIZE; ++i) {
        waveTable[i] = 2.0 - 2.0 * (i / (double)WAVE_TABLE_SIZE);
    }
}

int audioCallback(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status, void* userData) {
    HammerSound* hammerSound = static_cast<HammerSound*>(userData);
    return hammerSound->processAudioOutput(outputBuffer, inputBuffer, nBufferFrames, streamTime, status);
}

void HammerSound::sendTestBeep() {
    int testMidiPitch = 69; // A4 (440 Hz)
    int testMidiVelocity = 100;
    int testBeepDuration = 1000;

    sendNoteOn(testMidiPitch, testMidiVelocity);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    sendNoteOff(testMidiPitch);
}

void HammerSound::run() {
    try {
        // Initialize Audio
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

        audioOut->openStream(&parameters, nullptr, RTAUDIO_FLOAT64, sampleRate, &bufferFrames, &audioCallback, this);
        audioOut->startStream();

        sendTestBeep();
        
        auto midiOut = std::make_shared<RtMidiOut>();
        
        int num_ports = midiOut->getPortCount();
        std::string port_name;
        for (int i = 0; i < num_ports; i++) {
            try {
                port_name = midiOut->getPortName(i);
                std::cout << "MIDI port #" << i << ": " << port_name << std::endl;
            }
            catch (RtMidiError& error) {
                error.printMessage();
            }
        }

        if (midiOut->getPortCount() == 0) {
            std::cout << "No MIDI output ports available!" << std::endl;
            return;
        }
        midiOut->openPort(1); 
        std::cout << "MIDI output opened on port: " << midiOut->getPortName(1) << std::endl;
        
        char input;
        std::cout << "Press any key and hit enter to quit." << std::endl;
        std::cin >> input;

        midiOut->closePort();
        audioOut->closeStream();
    }
    
    catch (RtMidiError& error) {
        std::cerr << "RtMidi error: " << error.getMessage() << std::endl;
        return;
    }
    catch (...) {
        std::cerr << "Unknown error occurred while initializing audio and MIDI.\n";
        return;
    }
}

// SendNoteOn function
void HammerSound::sendNoteOn(int midi_pitch, int midi_velocity) {
    float frequency = 440.0f * powf(2.0f, (midi_pitch - 69) / 12.0f);
    float amplitude = (midi_velocity / 127.0f) * 0.3;

    std::unique_lock<std::mutex> lock(activeNotesMutex);

    bool slotFound = false;
    for (ActiveNote& note : activeNotes) {
        if (!note.adsrEnvelope.isActive()) {
            note.reset(frequency, amplitude); 
            note.adsrEnvelope.noteOn();
            slotFound = true;
            break;
        }
    }
    if (!slotFound) {
        activeNotes.push_back({ frequency, amplitude, 0.0, 0.0, true, 0.0, adsrEnvelope });
        activeNotes.back().adsrEnvelope.noteOn();
    }
    lock.unlock();
    std::cout << "Note ON: Midi pitch = " << midi_pitch << ", Midi velocity = " << midi_velocity << ", Frequency = " << frequency << " Hz\n";
    
    std::vector<unsigned char> noteOnMessage;
    noteOnMessage.push_back(0x90); 
    noteOnMessage.push_back(midi_pitch); 
    noteOnMessage.push_back(midi_velocity); 
    midiOut->sendMessage(&noteOnMessage);
}

// SendNoteOff function
void HammerSound::sendNoteOff(int midi_pitch) {
    float frequency = 440.0f * powf(2.0f, (midi_pitch - 69) / 12.0f);
 
    for (ActiveNote& note : activeNotes) {
        if (note.frequency == frequency) {
            note.adsrEnvelope.noteOff();
        }
    }
    std::cout << "Note OFF: Midi pitch = " << midi_pitch << " \n";

    std::vector<unsigned char> noteOffMessage;
    noteOffMessage.push_back(0x80); 
    noteOffMessage.push_back(midi_pitch); 
    noteOffMessage.push_back(0); 
    midiOut->sendMessage(&noteOffMessage);  
}

ADSR::ADSR(double attackTime, double decayTime, double sustainLevel, double releaseTime)
    : attackTime(attackTime), decayTime(decayTime), sustainLevel(sustainLevel), releaseTime(releaseTime), time(0.0f), state(State::Off) {}
bool ADSR::isActive() const {
    return state != State::Off;
}

void ADSR::noteOn() {
    state = State::Attack;
    time = 0.0;
}

void ADSR::noteOff() {
    if (state != State::Off) {
        state = State::Release;
        time = 0.0;
    }
}

void ADSR::reset() {
    state = State::Off;
    time = 0.0;
}

double ADSR::process(unsigned int sampleRate) {
    double output = 0.0;

    if (state == State::Attack) {
        output = time / attackTime;
        if (time >= attackTime) {
            state = State::Decay;
            time = 0.0;
        }
    }
    else if (state == State::Decay) {
        output = 1.0 - (1.0 - sustainLevel) * time / decayTime;
        if (time >= decayTime) {
            state = State::Sustain;
        }
    }
    else if (state == State::Sustain) {
        output = sustainLevel;
    }
    else if (state == State::Release) {
        output = sustainLevel * (1.0 - time / releaseTime);
        if (time >= releaseTime) {
            state = State::Off;
        }
    }

    time += 1.0 / sampleRate;
    return output;
}

SimpleCompressor::SimpleCompressor(double threshold, double ratio, double attack, double release, double kneeWidth)
    : threshold(threshold), ratio(ratio), attack(attack), release(release), kneeWidth(kneeWidth), envelope(0.0f) {}

double SimpleCompressor::processSample(double input) {
    double inputLevel = fabs(input);
    double gainReduction = 1.0;

    if (inputLevel > threshold) {
        gainReduction = 1.0 - ((1.0 - (threshold / inputLevel)) / ratio);
    }
    else if (inputLevel > threshold - kneeWidth / 2.0f && inputLevel <= threshold) {
        // Knee calculation
        double kneeLevel = (inputLevel - (threshold - kneeWidth / 2.0)) / kneeWidth;
        gainReduction = 1.0 - (1.0 / ratio) * kneeLevel * kneeLevel;
    }

    double desiredEnvelope = gainReduction;
    double attackRelease = inputLevel > envelope ? attack : release;
    envelope += (desiredEnvelope - envelope) * attackRelease;

    return input * envelope;
}

SimpleDelay::SimpleDelay(double delayTime, double feedback, double mix, unsigned int sampleRate)
    : delayBufferSize(static_cast<unsigned int>(delayTime* sampleRate)),
    delayBuffer(delayBufferSize, 0.0),
    delayWriteIndex(0),
    feedback(feedback),
    mix(mix) {}

double SimpleDelay::processSample(double inputSample) {
    unsigned int delayReadIndex = (delayWriteIndex - delayBufferSize) % delayBufferSize;
    double delaySample = delayBuffer[delayReadIndex];
    double outputSample = inputSample + mix * delaySample;
    delayBuffer[delayWriteIndex] = inputSample + feedback * delaySample;
    delayWriteIndex = (delayWriteIndex + 1) % delayBufferSize;
    return outputSample;
}

MasterGain::MasterGain(double gain) : gain(gain) {}

void MasterGain::setGain(double newGain) {
    gain = newGain;
}

double MasterGain::getGain() const {
    return gain;
}

double MasterGain::processSample(double inputSample) {
    return inputSample * gain;
}

// Process Audio Output
int HammerSound::processAudioOutput(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status) {

    double* outBuffer = static_cast<double*>(outputBuffer);
    unsigned int sampleRate = 44100;
    double fadeInTime = 0.01;  
    double sineWaveLevel = 0.5;   // (0.0 to 1.0)
    double squareWaveLevel = 0.3; 
    double sawtoothWaveLevel = 0.1;
    double triangleWaveLevel = 0.2;

    std::unique_lock<std::mutex> lock(activeNotesMutex);
    for (unsigned int i = 0; i < nBufferFrames; ++i) {
        double mixedSample = 0.0;

        for (ActiveNote& note : activeNotes) {
            double phaseRatio = note.phase / (2.0 * M_PI);
            unsigned int tableIndex = static_cast<unsigned int>(WAVE_TABLE_SIZE * phaseRatio) % WAVE_TABLE_SIZE;
            double carrierFrequency = note.frequency;

            double envelopeAmplitude = note.amplitude * note.adsrEnvelope.process(sampleRate);
           
            double fadeInFactor = std::min(note.time / fadeInTime, 1.0);
            double sineSample = sineWaveTable[tableIndex] * (1.0 - fadeInFactor);
            double squareSample = squareWaveTable[tableIndex] * fadeInFactor;
            double sawtoothSample = sawtoothWaveTable[tableIndex] * fadeInFactor;
            double triangleSample = triangleWaveTable[tableIndex] * fadeInFactor;
            mixedSample += envelopeAmplitude * (
                sineWaveLevel * sineSample +
                squareWaveLevel * squareSample +
                sawtoothWaveLevel * sawtoothSample +
                triangleWaveLevel * triangleSample
                );

            note.phase += (2.0 * M_PI * carrierFrequency) / sampleRate;
            note.time += 1.0 / sampleRate;
        }

        mixedSample = compressor.processSample(mixedSample);
        mixedSample = delay.processSample(mixedSample);
        mixedSample = masterGain.processSample(mixedSample);

        outBuffer[i * 2] = mixedSample; // Left channel
        outBuffer[i * 2 + 1] = mixedSample; // Right channel

    }
    lock.unlock();

    return 0;
}

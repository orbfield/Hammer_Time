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
    adsrEnvelope(/*attack*/0.002, /*decay*/0.01, /*sustain*/1.0, /*release*/0.008),
    compressor(/*threshold=*/0.05, /*ratio=*/15.0, /*attack=*/0.001, /*release=*/0.010,/*kneeWidth=*/0.2),
    lowPassFilter(/*cutoffFrequency=*/20000.0, /*sampleRate=*/48000),
    delay(/*delayTime=*/0.15, /*feedback=*/0.2, /*mix=*/0.1, /*sampleRate=*/48000),
    masterGain(1.0/*dB*/),
    sineWaveLevel(0.4),
    squareWaveLevel(0.2),
    sawtoothWaveLevel(0.1),
    triangleWaveLevel(0.1) {
    midiOut = std::make_shared<RtMidiOut>();
    createSineWaveTable(sineWaveTable);
    createSquareWaveTable(squareWaveTable);
    createSawToothWaveTable(sawtoothWaveTable);
    createTriangleWaveTable(triangleWaveTable);
  
}

HammerSound::~HammerSound() {}

// WaveTables
void HammerSound::createSineWaveTable(std::vector<double>& waveTable) {
    waveTable.resize(WAVE_TABLE_SIZE);

    for (unsigned int i = 0; i < WAVE_TABLE_SIZE; ++i) {
        //double phaseOffset = M_PI / 2; // Change the phase by 90 degrees
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

void HammerSound::sendTestBeep() {
    int testMidiPitch = 69; // A4 (440 Hz)
    int testMidiVelocity = 100;
    int testBeepDuration = 1000;

    sendNoteOn(testMidiPitch, testMidiVelocity);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    sendNoteOff(testMidiPitch);
}

int audioCallback(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames,
    double streamTime, RtAudioStreamStatus status, void* userData) {
    HammerSound* hammerSound = static_cast<HammerSound*>(userData);
    return hammerSound->processAudioOutput(outputBuffer, inputBuffer, nBufferFrames, streamTime, status);
}
static void midiInputCallback(double timeStamp, std::vector<unsigned char>* message, void* userData) {
    HammerSound* hammerSound = static_cast<HammerSound*>(userData);

    if (message->size() < 2) {
        return;  // Ignore messages that are too short
    }

    //unsigned char statusByte = message->at(0);
   // unsigned char messageType = statusByte & 0xF0;  // Mask the lower nibble (channel information)
    unsigned char messageType = message->at(0) & 0xF0;
    unsigned char channel = message->at(0) & 0x0F;
    switch (messageType) {
    case 0x80: {  // Note Off
        if (message->size() < 3) return;  // Note Off messages must have at least 3 bytes
        int midi_pitch = message->at(1);
        int midi_velocity = message->at(2);
        hammerSound->sendNoteOff(midi_pitch);
        break;
    }
    case 0x90: {  // Note On
        if (message->size() < 3) return;  // Note On messages must have at least 3 bytes
        int midi_pitch = message->at(1);
        int midi_velocity = message->at(2);
        if (midi_velocity == 0) {
            // Some devices use Note On with velocity 0 for Note Off
            hammerSound->sendNoteOff(midi_pitch);
        }
        else {
            hammerSound->sendNoteOn(midi_pitch, midi_velocity);
        }
        break;
    }
    case 0xB0: {  // Control Change
        unsigned char controllerNumber = message->at(1);
        unsigned char controllerValue = message->at(2);
        if (controllerNumber >= 48 && controllerNumber <= 51) { // Fader CC range from 49 to 51
            double waveLevel = static_cast<double>(controllerValue) / 127.0;
            hammerSound->setWaveLevel(controllerNumber, waveLevel);
        }
        // Handle CC93 separately if it has a different function
        if (controllerNumber == 93) {
            double waveLevel = static_cast<double>(controllerValue) / 127.0;
            hammerSound->setWaveLevel(controllerNumber, waveLevel);
        }
        break;
    }

    case 0xE0: {  // Pitch Bend
        if (message->size() < 3) return;
        int lsb = message->at(1);
        int msb = message->at(2);
        int pitchBendValue = (msb << 7) + lsb;  // Combine LSB and MSB to get 14-bit value
        // Process pitch bend here
        break;
    }
    
    default: {
        // Ignore other message types
        break;
    }
    }
}

void HammerSound::run() {
    try {
        if (!initializeAudio()) {
            std::cerr << "Failed to initialize audio!" << std::endl;
            return;
        }

        if (!initializeMidiIn()) {
            std::cerr << "Failed to initialize MIDI input!" << std::endl;
            return;
        }

        if (!initializeMidiOut()) {
            std::cerr << "Failed to initialize MIDI output!" << std::endl;
            return;
        }

        sendTestBeep();

        char input;
        std::cout << "Press any key and hit enter to quit." << std::endl;
        std::cin >> input;

        midiOut->closePort();
        midiIn->closePort();
        audioOut->closeStream();
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return;
    }
    catch (...) {
        std::cerr << "Unknown error occurred while initializing audio and MIDI." << std::endl;
        return;
    }
}

bool HammerSound::initializeAudio() {
    try {
        audioOut = std::make_unique<RtAudio>();

        if (audioOut->getDeviceCount() < 1) {
            std::cout << "No audio devices found!" << std::endl;
            return false;
        }

        RtAudio::StreamParameters parameters;
        parameters.deviceId = audioOut->getDefaultOutputDevice();
        parameters.nChannels = 2;
        parameters.firstChannel = 0;
        unsigned int sampleRate = 48000;
        unsigned int bufferFrames = 256;

        RtAudio::DeviceInfo deviceInfo = audioOut->getDeviceInfo(parameters.deviceId);
        std::cout << "\033[32mDefault Output Device:\033[0m " << deviceInfo.name << std::endl;

        audioOut->openStream(&parameters, nullptr, RTAUDIO_FLOAT64, sampleRate, &bufferFrames, &audioCallback, this);
        audioOut->startStream();

        sendTestBeep();

        return true;
    }
    /*
    catch (const RtAudioError& e) {
        std::cerr << "Error initializing audio: " << e.getMessage() << std::endl;
        return false;
    }*/
    catch (const std::exception& e) {
        std::cerr << "Error initializing audio: " << e.what() << std::endl;
        return false;
    }
    catch (...) {
        std::cerr << "Unknown error occurred while initializing audio." << std::endl;
        return false;
    }
}

bool HammerSound::initializeMidiIn() {
    try {
        midiIn = std::make_shared<RtMidiIn>();
        midiIn->setCallback(&midiInputCallback, this);

        if (midiIn->getPortCount() == 0) {
            std::cout << "No MIDI input ports available!" << std::endl;
            return false;
        }

        midiIn->openPort(2); // Replace 0 with the desired input port index
        std::cout << "MIDI input opened on port: " << midiIn->getPortName(2) << std::endl;
        return true;
    }
    catch (RtMidiError& error) {
        std::cerr << "RtMidi error: " << error.getMessage() << std::endl;
        return false;
    }
}

bool HammerSound::initializeMidiOut() {
    try {
        midiOut = std::make_shared<RtMidiOut>();

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
            return false;
        }

        midiOut->openPort(1); // Replace 1 with the desired output port index
        std::cout << "MIDI output opened on port: " << midiOut->getPortName(1) << std::endl;
        return true;
    }
    catch (RtMidiError& error) {
        std::cerr << "RtMidi error: " << error.getMessage() << std::endl;
        return false;
    }
}


// SendNoteOn function
void HammerSound::sendNoteOn(int midi_pitch, int midi_velocity) {
    double frequency = 440.0 * pow(2.0, (midi_pitch - 69) / 12.0);
    double amplitude = (midi_velocity / 127.0) * 0.3;

    std::unique_lock<std::mutex> lock(activeNotesMutex);

    bool slotFound = false;
    for (ActiveNote& note : activeNotes) {
        if (!note.adsrEnvelope.isActive()) {
            note.reset(frequency, amplitude); 
            note.adsrEnvelope.noteOn();
            note.phase = 0.0;
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
    double frequency = 440.0 * pow(2.0, (midi_pitch - 69) / 12.0);
 
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
        double nextOutput = (time + 1.0 / sampleRate) / attackTime;
        output = time / attackTime;
        output += (nextOutput - output) * (time - floor(time));

        if (time >= attackTime) {
            state = State::Decay;
            time = 0.0;
        }
    }
    else if (state == State::Decay) {
        double nextOutput = 1.0 - (1.0 - sustainLevel) * (time + 1.0 / sampleRate) / decayTime;
        output = 1.0 - (1.0 - sustainLevel) * time / decayTime;
        output += (nextOutput - output) * (time - floor(time));

        if (time >= decayTime) {
            state = State::Sustain;
        }
    }
    else if (state == State::Sustain) {
        output = sustainLevel;
    }
    else if (state == State::Release) {
        double nextOutput = sustainLevel * (1.0 - (time + 1.0 / sampleRate) / releaseTime);
        output = sustainLevel * (1.0 - time / releaseTime);
        output += (nextOutput - output) * (time - floor(time));

        if (time >= releaseTime) {
            state = State::Off;
        }
    }

    time += 1.0 / sampleRate;
    return output;
}


SimpleCompressor::SimpleCompressor(double threshold, double ratio, double attack, double release, double kneeWidth)
    : threshold(threshold), ratio(ratio), attack(attack), release(release), kneeWidth(kneeWidth), envelope(0.0f), rmsSum(0.0), rmsWindowSize(100) {}

double SimpleCompressor::processSample(double input) {
    // RMS envelope follower
    double square = input * input;
    rmsSum += square;
    rmsWindow.push_back(square);
    if (rmsWindow.size() > rmsWindowSize) {
        rmsSum -= rmsWindow.front();
        rmsWindow.pop_front();
    }
    double rms = sqrt(rmsSum / rmsWindow.size());

    double inputLevel = rms;
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

LowPassFilter::LowPassFilter(double cutoffFrequency, double sampleRate)
    : previousOutput(0.0) {
    double RC = 1.0 / (cutoffFrequency * 2.0 * M_PI);
    double dt = 1.0 / sampleRate;
    alpha = dt / (RC + dt);
}

double LowPassFilter::processSample(double inputSample) {
    double outputSample = alpha * inputSample + (1.0 - alpha) * previousOutput;
    previousOutput = outputSample;
    return outputSample;
}


SimpleDelay::SimpleDelay(double delayTime, double feedback, double mix, unsigned int sampleRate)
    : delayBufferSize(static_cast<unsigned int>(delayTime* sampleRate)),
    delayBufferLeft(delayBufferSize, 0.0),
    delayBufferRight(delayBufferSize, 0.0),
    delayWriteIndexLeft(0),
    delayWriteIndexRight(0),
    feedback(feedback),
    mix(mix) {}

double SimpleDelay::processSample(double inputSample, bool isLeftChannel) {
    double delayTimeFraction = delayBufferSize - floor(delayBufferSize);
    unsigned int delayWriteIndex = isLeftChannel ? delayWriteIndexLeft : delayWriteIndexRight;
    unsigned int delayReadIndex = (delayWriteIndex - static_cast<unsigned int>(floor(delayBufferSize))) % delayBufferSize;
    unsigned int nextDelayReadIndex = (delayReadIndex + 1) % delayBufferSize;

    std::vector<double>& delayBuffer = isLeftChannel ? delayBufferLeft : delayBufferRight;
    std::vector<double>& otherDelayBuffer = isLeftChannel ? delayBufferRight : delayBufferLeft;

    double delaySample1 = otherDelayBuffer[delayReadIndex];
    double delaySample2 = otherDelayBuffer[nextDelayReadIndex];

    double delaySample = delaySample1 * (1.0 - delayTimeFraction) + delaySample2 * delayTimeFraction;
    double outputSample = inputSample + mix * delaySample;
    delayBuffer[delayWriteIndex] = inputSample + feedback * delaySample;

    if (isLeftChannel) {
        delayWriteIndexLeft = (delayWriteIndexLeft + 1) % delayBufferSize;
    }
    else {
        delayWriteIndexRight = (delayWriteIndexRight + 1) % delayBufferSize;
    }

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
    unsigned int sampleRate = 48000;
    double sineWaveLevel;
    double squareWaveLevel;
    double sawtoothWaveLevel;
    double triangleWaveLevel;

    {
        std::lock_guard<std::mutex> lock(mtx);
        sineWaveLevel = this->sineWaveLevel;
        squareWaveLevel = this->squareWaveLevel;
        sawtoothWaveLevel = this->sawtoothWaveLevel;
        triangleWaveLevel = this->triangleWaveLevel;
    }

    std::unique_lock<std::mutex> lock(activeNotesMutex);
    for (unsigned int i = 0; i < nBufferFrames; ++i) {
        double mixedSample = 0.0;

        for (ActiveNote& note : activeNotes) {
            double phaseRatio = note.phase / (2.0 * M_PI);
            unsigned int tableIndex = static_cast<unsigned int>(WAVE_TABLE_SIZE * phaseRatio) % WAVE_TABLE_SIZE;
            double carrierFrequency = note.frequency;

            double envelopeAmplitude = note.amplitude * note.adsrEnvelope.process(sampleRate);

            double t = WAVE_TABLE_SIZE * phaseRatio - floor(WAVE_TABLE_SIZE * phaseRatio);
            unsigned int nextTableIndex = (tableIndex + 1) % WAVE_TABLE_SIZE;

            double sineSample = sineWaveTable[tableIndex] * (1.0 - t) + sineWaveTable[nextTableIndex] * t;
            double squareSample = squareWaveTable[tableIndex] * (1.0 - t) + squareWaveTable[nextTableIndex] * t;
            double sawtoothSample = sawtoothWaveTable[tableIndex] * (1.0 - t) + sawtoothWaveTable[nextTableIndex] * t;
            double triangleSample = triangleWaveTable[tableIndex] * (1.0 - t) + triangleWaveTable[nextTableIndex] * t;


            mixedSample += envelopeAmplitude * (
                sineWaveLevel * sineSample +
                squareWaveLevel * squareSample +
                sawtoothWaveLevel * sawtoothSample +
                triangleWaveLevel * triangleSample
                );

            note.phase += (2.0 * M_PI * carrierFrequency) / sampleRate;
            note.time += 1.0 / sampleRate;
        }

        mixedSample = lowPassFilter.processSample(mixedSample);
        mixedSample = compressor.processSample(mixedSample);
        
        double leftSample = delay.processSample(mixedSample, true); // Left channel
        double rightSample = delay.processSample(mixedSample, false); // Right channel
        outBuffer[i * 2] = masterGain.processSample(leftSample); 
        outBuffer[i * 2 + 1] = masterGain.processSample(rightSample); 


    }
    lock.unlock();

    return 0;
}

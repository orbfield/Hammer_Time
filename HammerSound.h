#pragma once
#include <RtAudio.h>
#include <mutex>
#include <C:/Dev/Iibaries/rtmidi-5.0.0/RtMidi.h>
#include <atomic>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <deque>


const unsigned int WAVE_TABLE_SIZE = 4096;

class ADSR {
public:
    ADSR(double attackTime, double decayTime, double sustainLevel, double releaseTime);
    bool isActive() const;
    void noteOn();
    void noteOff();
    void reset();
    double process(unsigned int sampleRate);

private:
    enum class State { Off, Attack, Decay, Sustain, Release };
    State state;
    double attackTime;
    double decayTime;
    double sustainLevel;
    double releaseTime;
    double time;
};


struct ActiveNote {
    void reset(double newFrequency, double newAmplitude) {
        frequency = newFrequency;
        amplitude = newAmplitude;
        phase = 0.0;
        time = 0.0;
        isNoteOn = true;
        originalPhase = 0.0;
        adsrEnvelope.reset(); 
    }
    double frequency;
    double amplitude;
    double phase;
    double time;
    bool isNoteOn;
    double originalPhase;
    
    ADSR adsrEnvelope;
};


class SimpleCompressor {
public:
    SimpleCompressor(double threshold, double ratio, double attack, double release, double kneeWidth);

    double processSample(double input);

private:
    double threshold;
    double ratio;
    double attack;
    double release;
    double kneeWidth;
    double envelope;
    double rmsSum;
    unsigned int rmsWindowSize;
    std::deque<double> rmsWindow;
};

class LowPassFilter {
public:
    LowPassFilter(double cutoffFrequency, double sampleRate);

    double processSample(double inputSample);

private:
    double alpha;
    double previousOutput;
};


class SimpleDelay {
public:
    SimpleDelay(double delayTime, double feedback, double mix, unsigned int sampleRate);

    double processSample(double inputSample, bool isLeftChannel);

private:
    unsigned int delayBufferSize;
    std::vector<double> delayBufferLeft;
    std::vector<double> delayBufferRight;
    unsigned int delayWriteIndexLeft;
    unsigned int delayWriteIndexRight;
    double feedback;
    double mix;
};

class MasterGain {
public:
    MasterGain(double gain);

    void setGain(double newGain);
    double getGain() const;
    double processSample(double inputSample);

private:
    double gain;
};

class HammerSound {
public:
    HammerSound();
    ~HammerSound();
    void createSineWaveTable(std::vector<double>& waveTable);
    void createSquareWaveTable(std::vector<double>& waveTable);
    void createSawToothWaveTable(std::vector<double>& waveTable);
    void createTriangleWaveTable(std::vector<double>& waveTable);
    void run();
    int processAudioOutput(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status);
    void sendNoteOn(int midi_pitch, int midi_velocity);
    void sendNoteOff(int midi_pitch);
    void sendTestBeep();
    void setWaveLevel(unsigned char controllerNumber, double waveLevel) {
        std::lock_guard<std::mutex> lock(mtx);
        switch (controllerNumber) {
        case 49:  // Fader 1
            sineWaveLevel = waveLevel;
            break;
        case 50:  // Fader 2
            squareWaveLevel = waveLevel;
            break;
        case 51:  // Fader 3
            sawtoothWaveLevel = waveLevel;
            break;
        case 93:  // Fader 4
            triangleWaveLevel = waveLevel;
            break;
        default:
            break;
        }
    }


private:
    std::unique_ptr<RtAudio> audioOut;
    std::shared_ptr<RtMidiOut> midiOut;
    std::shared_ptr<RtMidiIn> midiIn;
    std::mutex mtx;

    bool initializeAudio();
    bool initializeMidiIn();
    bool initializeMidiOut();

    std::atomic<bool> isPlaying;
    std::vector<ActiveNote> activeNotes;
    std::mutex activeNotesMutex;
    std::mutex midiOutMutex;
    

    std::atomic<double> currentFrequency;
    std::atomic<double> amplitude;
    std::vector<double> sineWaveTable;
    std::vector<double> squareWaveTable;
    std::vector<double> sawtoothWaveTable;
    std::vector<double> triangleWaveTable;
    std::atomic<double> sineWaveLevel;
    std::atomic<double> squareWaveLevel;
    std::atomic<double> sawtoothWaveLevel;
    std::atomic<double> triangleWaveLevel;

    SimpleCompressor compressor;
    ADSR adsrEnvelope;
    LowPassFilter lowPassFilter;
    SimpleDelay delay;
    MasterGain masterGain;  
};




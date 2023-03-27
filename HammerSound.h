#pragma once
#include <RtAudio.h>
#include <mutex>
#include <C:/Dev/Iibaries/rtmidi-5.0.0/RtMidi.h>
#include <atomic>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>


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
    void reset(float newFrequency, float newAmplitude) {
        frequency = newFrequency;
        amplitude = newAmplitude;
        phase = 0.0f;
        time = 0.0f;
        isNoteOn = true;
        originalPhase = 0.0f;
        adsrEnvelope.reset(); 
    }
    float frequency;
    float amplitude;
    float phase;
    float time;
    bool isNoteOn;
    float originalPhase;
    
    ADSR adsrEnvelope;
};

const unsigned int WAVE_TABLE_SIZE = 2048;


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
};

class SimpleDelay {
public:
    SimpleDelay(double delayTime, double feedback, double mix, unsigned int sampleRate);

    double processSample(double inputSample);

private:
    unsigned int delayBufferSize;
    std::vector<double> delayBuffer;
    unsigned int delayWriteIndex;
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


private:
    std::unique_ptr<RtAudio> audioOut;
    std::shared_ptr<RtMidiOut> midiOut;
    std::mutex mtx;

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

    SimpleCompressor compressor;
    ADSR adsrEnvelope;
    SimpleDelay delay;
    MasterGain masterGain;
};




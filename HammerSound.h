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

   

    void setMidiInputDeviceName(const std::string& name) {
        midiInputDeviceName = name;
    }

    void setPitchBend(double value) {
        std::lock_guard<std::mutex> lock(pitchBendMtx);
        pitchBend = value;
    }
    
    // Add a new method to handle fader 5 value changes
    void Fader5PitchBend(double value) {
        // Normalize value to the pitch bend range of -1 to 1
        double pitchBendValue = 2.0 * (value - 0.5);
        setPitchBend(pitchBendValue);
    }
    double getPitchBend() const {
        std::lock_guard<std::mutex> lock(pitchBendMtx);
        return pitchBend;
    }

    void sendFaderValue(unsigned char channel, double value);
 
    void setWaveLevel(unsigned char channel, double waveLevel) {
        std::lock_guard<std::mutex> lock(mtx);
        switch (channel) {
        case 0:  // Fader 1
            sineWaveLevel = waveLevel;
            break;
        case 1:  // Fader 2
            squareWaveLevel = waveLevel;
            break;
        case 2:  // Fader 3
            sawtoothWaveLevel = waveLevel;
            break;
        case 3:  // Fader 4
            triangleWaveLevel = waveLevel;
            break;

        default:
            break;
        }
    }
    void initializeFaders() {
        sendFaderValue(0, sineWaveLevel * 16383.0);
        sendFaderValue(1, squareWaveLevel * 16383.0);
        sendFaderValue(2, sawtoothWaveLevel * 16383.0);
        sendFaderValue(3, triangleWaveLevel * 16383.0);
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

    double pitchBend; // Pitch bend value, range: [-1, 1]
    mutable std::mutex pitchBendMtx;

    std::string midiInputDeviceName; 
    /*bool isEuphonixMCMix() const {
        return midiInputDeviceName.find("Euphonix") != std::string::npos;
    }*/

    SimpleCompressor compressor;
    ADSR adsrEnvelope;
    LowPassFilter lowPassFilter;
    SimpleDelay delay;
    MasterGain masterGain;  
};




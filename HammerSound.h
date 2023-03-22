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
    ADSR(double attackTime, double decayTime, double sustainLevel, double releaseTime)
        : attackTime(attackTime), decayTime(decayTime), sustainLevel(sustainLevel), releaseTime(releaseTime), time(0.0), state(State::Off) {}

    void noteOn() {
        state = State::Attack;
        time = 0.0;
    }

    void noteOff() {
        if (state != State::Off) {
            state = State::Release;
            time = 0.0;
        }
    }

    double process(double sampleRate) {
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
    double frequency;
    double amplitude;
    double phase;
    double time;
    bool isNoteOn;
    double fadeTime;
    ADSR adsrEnvelope;

    double duration;

    bool fadingOut;
    double originalPhase;
};

const unsigned int WAVE_TABLE_SIZE = 2048;

class SimpleCompressor {
public:
    SimpleCompressor(double threshold, double ratio, double attack, double release)
        : threshold(threshold), ratio(ratio), attack(attack), release(release), envelope(0.0) {}

    double processSample(double input) {
        double inputLevel = fabs(input);
        double gainReduction = 1.0;

        if (inputLevel > threshold) {
            gainReduction = threshold / inputLevel;
            double desiredEnvelope = gainReduction;
            double attackRelease = inputLevel > envelope ? attack : release;
            envelope += (desiredEnvelope - envelope) * attackRelease;
        }

        return input * envelope;
    }

private:
    double threshold;
    double ratio;
    double attack;
    double release;
    double envelope;
};



class HammerSound {
public:
    HammerSound();
    ~HammerSound();
    void createSineWaveTable(std::vector<double>& waveTable);
    void run();
    int processAudioOutput(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status);
    void sendNoteOn(int midi_pitch, int midi_velocity);
    void sendNoteOff(int midi_pitch);
    void sendTestBeep();
    double raisedCosineFadeIn(double time, double duration);
  

private:
    std::unique_ptr<RtAudio> audioOut;
    std::mutex mtx;

    std::atomic<bool> isPlaying;
    std::vector<ActiveNote> activeNotes;
    std::mutex activeNotesMutex;

    std::atomic<double> currentFrequency;
    std::atomic<double> amplitude;
    std::vector<double> sineWaveTable;

    SimpleCompressor compressor;
    ADSR adsrEnvelope;

};


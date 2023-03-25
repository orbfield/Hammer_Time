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
    ADSR(float attackTime, float decayTime, float sustainLevel, float releaseTime)
        : attackTime(attackTime), decayTime(decayTime), sustainLevel(sustainLevel), releaseTime(releaseTime), time(0.0f), state(State::Off) {}

    void noteOn() {
        state = State::Attack;
        time = 0.0f;
    }

    void noteOff() {
        if (state != State::Off) {
            state = State::Release;
            time = 0.0f;
        }
    }

    float process(float sampleRate) {
        float output = 0.0f;

        if (state == State::Attack) {
            output = time / attackTime;
            if (time >= attackTime) {
                state = State::Decay;
                time = 0.0f;
            }
        }
        else if (state == State::Decay) {
            output = 1.0f - (1.0f - sustainLevel) * time / decayTime;
            if (time >= decayTime) {
                state = State::Sustain;
            }
        }
        else if (state == State::Sustain) {
            output = sustainLevel;
        }
        else if (state == State::Release) {
            output = sustainLevel * (1.0f - time / releaseTime);
            if (time >= releaseTime) {
                state = State::Off;
            }
        }

        time += 1.0f / sampleRate;
        return output;
    }

private:
    enum class State { Off, Attack, Decay, Sustain, Release };
    State state;

    float attackTime;
    float decayTime;
    float sustainLevel;
    float releaseTime;
    float time;
};

struct ActiveNote {
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
    SimpleCompressor(float threshold, float ratio, float attack, float release)
        : threshold(threshold), ratio(ratio), attack(attack), release(release), envelope(0.0f) {}

    float processSample(float input) {
        float inputLevel = fabs(input);
        float gainReduction = 1.0f;

        if (inputLevel > threshold) {
            gainReduction = 1.0f - ((1.0f - (threshold / inputLevel)) / ratio);
            float desiredEnvelope = gainReduction;
            float attackRelease = inputLevel > envelope ? attack : release;
            envelope += (desiredEnvelope - envelope) * attackRelease;
        }

        return input * envelope;
    }

private:
    float threshold;
    float ratio;
    float attack;
    float release;
    float envelope;
};

class SimpleDelay {
public:
    SimpleDelay(float delayTime, float feedback, float mix, float sampleRate)
        : delayBufferSize(static_cast<unsigned int>(delayTime* sampleRate)),
        delayBuffer(delayBufferSize, 0.0f),
        delayWriteIndex(0),
        feedback(feedback),
        mix(mix) {}

    float processSample(float inputSample) {
        unsigned int delayReadIndex = (delayWriteIndex - delayBufferSize) % delayBufferSize;
        float delaySample = delayBuffer[delayReadIndex];
        float outputSample = inputSample + mix * delaySample;
        delayBuffer[delayWriteIndex] = inputSample + feedback * delaySample;
        delayWriteIndex = (delayWriteIndex + 1) % delayBufferSize;
        return outputSample;
    }

private:
    unsigned int delayBufferSize;
    std::vector<float> delayBuffer;
    unsigned int delayWriteIndex;
    float feedback;
    float mix;
};


class MasterGain {
public:
    MasterGain(float gain) : gain(gain) {}

    void setGain(float newGain) {
        gain = newGain;
    }

    float getGain() const {
        return gain;
    }

    double processSample(double inputSample) {
        return inputSample * gain;
    }

private:
    float gain;
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
    SimpleDelay delay;
    MasterGain masterGain;
};



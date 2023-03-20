#pragma once
#include <RtAudio.h>
#include <mutex>
#include <C:/Dev/Iibaries/rtmidi-5.0.0/RtMidi.h>
#include <atomic>

class HammerSound {
public:
    HammerSound();
    ~HammerSound();

    void run();
    int processAudioOutput(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status);
    void sendNoteOn(int midi_pitch, int midi_velocity);
    void sendNoteOff(int midi_pitch);
    void sendTestBeep();

private:
    std::unique_ptr<RtAudio> audioOut;
    std::mutex mtx;
    
    std::atomic<double> currentFrequency;
    std::atomic<double> amplitude;
    std::atomic<bool> isPlaying;
};
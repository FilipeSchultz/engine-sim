#include <gtest/gtest.h>

#include "../include/synthesizer.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace std::chrono_literals;

void setupStandardSynthesizer(Synthesizer &synth) {
    Synthesizer::Parameters params;
    params.audioBufferSize = 512 * 16;
    params.audioSampleRate = 16;
    params.inputBufferSize = 256;
    params.inputChannelCount = 8;
    params.inputSampleRate = 32;

    Synthesizer::AudioParameters audioParams;
    audioParams.airNoise = 0.0;
    audioParams.inputSampleNoise = 0.0;
    audioParams.levelerMaxGain = 1.0;
    audioParams.levelerMinGain = 1.0;
    audioParams.dF_F_mix = 0.0;
    params.initialAudioParameters = audioParams;

    synth.initialize(params);
}

void setupSynchronizedSynthesizer(Synthesizer &synth) {
    Synthesizer::Parameters params;
    params.audioBufferSize = 512 * 16;
    params.audioSampleRate = 32;
    params.inputBufferSize = 1024;
    params.inputChannelCount = 8;
    params.inputSampleRate = 32;

    Synthesizer::AudioParameters audioParams;
    audioParams.airNoise = 0.0;
    audioParams.inputSampleNoise = 0.0;
    audioParams.levelerMaxGain = 1.0;
    audioParams.levelerMinGain = 1.0;
    audioParams.dF_F_mix = 0.0;
    params.initialAudioParameters = audioParams;

    synth.initialize(params);
}

TEST(SynthesizerTests, SynthesizerSanityCheck) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);
    synth.destroy();
}
/*
TEST(SynthesizerTests, SynthesizerConversionTest) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    EXPECT_NEAR(synth.inputSampleToTimeOffset(0.0), 0.0, 1E-6);
    EXPECT_NEAR(synth.inputSampleToTimeOffset(1.0), 1 / 32.0, 1E-6);

    EXPECT_NEAR(synth.audioSampleToTimeOffset(0), -0.5, 1E-6);

    synth.destroy();
}

TEST(SynthesizerTests, SynthesizerTrimTest) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    const double timeOffset0 = synth.audioSampleToTimeOffset(0);

    synth.trimInput(0.5, false);

    const double timeOffset1 = synth.audioSampleToTimeOffset(8);

    EXPECT_NEAR(timeOffset1, timeOffset0, 1E-6);

    synth.destroy();
}

TEST(SynthesizerTests, SynthesizerSampleTest) {
    Synthesizer synth;
    setupStandardSynthesizer(synth);

    for (int i = 0; i < 1024; ++i) {
        const double v = (double)i;
        const double data[] = { v, v, v, v, v, v, v, v };
        synth.writeInput(data);
    }

    const double end_t = 1023 / 32.0;

    const double v0 = synth.sampleInput(end_t, 0);
    const double v1 = synth.sampleInput(end_t - 1 / 64.0, 0);

    EXPECT_NEAR(v0, 1023.0, 1E-6);
    EXPECT_NEAR(v1, 1022.5, 1E-6);

    synth.trimInput(0.5);

    const double v0_trim = synth.sampleInput(end_t, 0);
    const double v1_trim = synth.sampleInput(end_t - 1 / 64.0, 0);

    EXPECT_NEAR(v0, v0_trim, 1E-6);
    EXPECT_NEAR(v1, v1_trim, 1E-6);

    synth.destroy();
}
*/

namespace {
    // Unit impulse response on every channel: the convolution passes the
    // signal through (the synthesizer needs one before rendering)
    void setUnitImpulseResponses(Synthesizer &synth, int channels) {
        const int16_t impulse[] = { INT16_MAX };
        for (int i = 0; i < channels; ++i) {
            synth.initializeImpulseResponse(impulse, 1, 1.0f, i);
        }
    }

    // Half the input buffer (1024): the synthesizer is fed continuously in the
    // simulator, a test that stops feeding must not wrap the input buffer
    constexpr int SystemTestBlocks = 32;
    constexpr int SystemTestBlockSize = 16;

    double rampInput(int i, bool zero) {
        return zero ? 0.0 : 100.0 * std::sin(i * 0.2);
    }

    // Renders the test signal block by block in this thread
    std::vector<int16_t> renderSingleThread(bool zero) {
        Synthesizer synth;
        setupSynchronizedSynthesizer(synth);
        setUnitImpulseResponses(synth, 8);

        std::vector<int16_t> output;
        int16_t block[2048];
        for (int i = 0; i < SystemTestBlocks * SystemTestBlockSize;) {
            for (int j = 0; j < SystemTestBlockSize; ++j, ++i) {
                const double v = rampInput(i, zero);
                const double data[] = { v, v, v, v, v, v, v, v };
                synth.writeInput(data);
            }

            synth.endInputBlock();
            synth.renderAudio();

            const int n = synth.readAudioOutput(2048, block);
            output.insert(output.end(), block, block + n);
        }

        synth.destroy();
        return output;
    }
}

TEST(SynthesizerTests, SynthesizerSystemTestSingleThread) {
    // Same input and output rate: one audio sample per input sample (after
    // the initial silence of the audio buffer)
    const std::vector<int16_t> silent = renderSingleThread(true);
    const std::vector<int16_t> signal = renderSingleThread(false);

    ASSERT_EQ(silent.size(), signal.size());
    EXPECT_NEAR((double)signal.size(), SystemTestBlocks * SystemTestBlockSize, 2.0);

    // No input, no sound (no air noise, no jitter)
    for (int16_t v : silent) EXPECT_EQ(v, 0);

    // A varying input is heard
    int nonZero = 0;
    for (int16_t v : signal) if (v != 0) ++nonZero;
    EXPECT_GT(nonZero, (int)signal.size() / 2);
}

TEST(SynthesizerTests, SynthesizerSystemTest) {
    // The rendering thread gives the same audio as rendering in this thread
    const std::vector<int16_t> reference = renderSingleThread(false);

    Synthesizer synth;
    setupSynchronizedSynthesizer(synth);
    setUnitImpulseResponses(synth, 8);
    synth.startAudioRenderingThread();

    std::vector<int16_t> output;
    int16_t block[2048];
    for (int i = 0; i < SystemTestBlocks * SystemTestBlockSize;) {
        for (int j = 0; j < SystemTestBlockSize; ++j, ++i) {
            const double v = rampInput(i, false);
            const double data[] = { v, v, v, v, v, v, v, v };
            synth.writeInput(data);
        }

        synth.endInputBlock();
        synth.waitProcessed();

        const int n = synth.readAudioOutput(2048, block);
        output.insert(output.end(), block, block + n);
    }

    // The last block may still be rendering: stop the thread (it finishes
    // the block) and read the rest
    synth.endAudioRenderingThread();
    const int n = synth.readAudioOutput(2048, block);
    output.insert(output.end(), block, block + n);
    synth.destroy();

    ASSERT_EQ(output.size(), reference.size());
    for (size_t i = 0; i < output.size(); ++i) {
        EXPECT_EQ(output[i], reference[i]) << "sample " << i;
    }
}

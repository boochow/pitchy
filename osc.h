#pragma once
/*
 *  File: osc.h
 *
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <climits>

#include "unit_osc.h"   // Note: Include base definitions for osc units
#include "osc_api.h" // for osc_notehzf()
#include "utils/int_math.h"   // for clipminmaxi32()

constexpr uint8_t base_note = 53;
// max buffer size = 32KB = 4096samples * float(4bytes) * 2 channels
// 4096 samples = 85.3 msec
constexpr size_t max_buffer_size = 1 << 13;

inline float note2freq(float note) {
    return (440.f / 32) * powf(2, (note - 9.0) / 12);
}

class Osc {
public:
    enum {
        SHAPE = 0U,
        ALT,
        GRAIN_SIZE,
        GRAIN_DEPTH,
        PITCH_LOW,
        PITCH_HIGH,
        GATE,
        MIX_SOURCE,
        SOURCE_GATE,
        NUM_PARAMS
    };

    Osc(void) {}
    ~Osc(void) {}

    inline int8_t Init(const unit_runtime_desc_t * desc) {
        if (!desc)
            return k_unit_err_undef;
    
        if (desc->target != unit_header.target)
            return k_unit_err_target;
    
        if (!UNIT_API_IS_COMPAT(desc->api))
            return k_unit_err_api_version;

        if (desc->samplerate != 48000)
            return k_unit_err_samplerate;

        if (desc->input_channels != 2 || desc->output_channels != 1)  // should be stereo input / mono output
            return k_unit_err_geometry;

        runtime_desc_ = *desc;
    
        phi_ = 0;
        note_ = 0;
        input_gain_ = 128.f;
        w0_ = 1.0f;

        return k_unit_err_none;
    }

    inline void Teardown() {
    }

    inline void Reset() {
        phi_ = 0;
        note_ = 0;
        input_gain_ = 128.f;
    }

    inline void Resume() {
    }

    inline void Suspend() {
    }

    fast_inline void Process(const float * in, float * out, size_t frames) {
        const float * __restrict in_p = in;
        float * __restrict out_p = out;
        const float * out_e = out_p + frames;  // assuming mono output

        /*
        const unit_runtime_osc_context_t *ctxt = static_cast<const unit_runtime_osc_context_t *>(runtime_desc_.hooks.runtime_context);
        const float w0 = osc_w0f_for_note((ctxt->pitch)>>8, ctxt->pitch & 0xFF);
        */
//        const float w0 = osc_w0f_for_note(pitch >> 8, pitch & 0xff);

        for (; out_p != out_e; in_p += 2, out_p += 1) {
            float sig = *in_p + *(in_p + 1);

            delay_write(sig, grain_delay_depth_);
            sig = delay_read(phi_);
            phi_ += w0_;
            if (((int) phi_) >= buffer_size_) {
                phi_ -= buffer_size_;
            }
//            *(out_p) = sig * input_gain_ + (256. - input_gain_) * (*in_p + *(in_p + 1));
            *(out_p) = sig * input_gain_;
        }
    }

    inline void setParameter(uint8_t index, int32_t value) {
        switch(index) {
        case SHAPE:
            w0_ = value * 0.001 + 0.75f;
            break;
        case ALT:
            input_gain_ = 0.25f * value;
            break;
        case GRAIN_SIZE:
            if (value > 8) {
                value = 8;
            }
            if (value != p_[GRAIN_SIZE]) {
                buffer_size_ = 1 << (13 - value);
                buffer_mask_ = buffer_size_ - 1;
                delay_pos_ = 2;
                phi_ = 0.f;
            }
            break;
        case GRAIN_DEPTH:
            grain_delay_depth_ = 0.01 * value;
            break;
        default:
            break;
        }
        p_[index] = value;
    }

    inline int32_t getParameterValue(uint8_t index) const {
        return 0;
    }

    inline const char * getParameterStrValue(uint8_t index, int32_t value) const {
        return nullptr;
    }

    inline void setTempo(uint32_t tempo) {
        (uint32_t)tempo;
    }

    inline void tempo4ppqnTick(uint32_t counter) {
        (uint32_t)counter;
    }

    inline void NoteOn(uint8_t note, uint8_t velo) {
        note_ = note;
        gate_ = true;
//        float base_hz = osc_notehzf(base_note);
//        float note_hz = osc_notehzf(note);
        float base_hz = note2freq(base_note);
        float note_hz = note2freq(note);
        w0_ = note_hz / base_hz;
        phi_ = delay_pos_ - 4;
        if (phi_ < 0) {
            phi_ += buffer_size_;
        }
    }

    inline void NoteOff(uint8_t note) {
        (uint8_t)note;
        gate_ = false;
    }

    inline void AllNoteOff() {
    }

    inline void PitchBend(uint16_t bend) {
        (uint16_t)bend;
    }

    inline void ChannelPressure(uint8_t press) {
        (uint8_t)press;
    }

    inline void AfterTouch(uint8_t note, uint8_t press) {
        (uint8_t)note;
        (uint8_t)press;
    }

private:

    unit_runtime_desc_t runtime_desc_;

    int32_t p_[11];
    float phi_;
    uint8_t note_;
    bool gate_;
    float input_gain_;
    float w0_;
    float delay_line_[max_buffer_size];
    size_t buffer_size_ = max_buffer_size;
    size_t buffer_mask_ = buffer_size_ - 1;
    float grain_delay_depth_;
    int32_t delay_pos_;

    void delay_write(const float sig, const float mix) {
        delay_line_[delay_pos_++] = sig + mix * delay_line_[delay_pos_];
	delay_pos_ = delay_pos_ & buffer_mask_;
    }

    float delay_read(float pos) {
	// Note: this code doesn't interpolate frac part of pos
	int32_t p = int(pos);
	return delay_line_[p];
    }
};

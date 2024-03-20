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

// the note number which plays grain in the normal pitch
constexpr uint8_t base_note = 53;
constexpr float base_hz = 174.61411571650194; // note2freq(base_note)
// max buffer size = 32KB = 8192samples * float(4bytes)
// (note: 8192 samples = 170.66 msec)
constexpr size_t max_buffer_size = 1 << 13;

inline float note2freq(float note) {
    return (440.f / 32) * powf(2, (note - 9.0) / 12);
}


class Osc {
public:
    enum {
        SHAPE = 0U,  // fine pitch
        ALT,         // input gain
        GRAIN_SIZE,  // the size of the grain
        GRAIN_DEPTH, // the mix level of "old" grain sound
        WIDTH_DOWN,  // the max transpose-down width for SHAPE parameter
        WIDTH_UP,    // the max transpose-up width for SHAPE parameter
        MIX_DRYWET,  // the mix level of the dry/wet sound
        GATE,        // if true, key off makes the osc off
        GATE_SOURCE, // if true, key off makes the input sound off
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
            float in_sig = *in_p + *(in_p + 1);

            delay_write(in_sig, grain_delay_depth_);
            float out_sig = delay_read(phi_);
            phi_ += w0_;
            if (((int) phi_) >= buffer_size_) {
                phi_ -= buffer_size_;
            }
            int32_t gate_wet = (gate_wet_) ? gate_ : 1;
            int32_t gate_dry = (gate_dry_) ? gate_ : 1;
            out_sig = wet_ * out_sig * gate_wet + dry_ * in_sig * gate_dry;
            *(out_p) = osc_softclipf(0.1f, out_sig * input_gain_);
        }
    }

    inline void setParameter(uint8_t index, int32_t value) {
        switch(index) {
        case SHAPE:
            w0_ = shape_w0_value( (value < 500) ? (value - 500) * 0.002 :
                                  (value > 523) ? (value - 523) * 0.002 : 0);
            break;
        case ALT:
            input_gain_ = 1.f * value / 170.5; // 0..6.0
            input_gain_ = 0.5f + input_gain_ * input_gain_; // 0.5..36.5
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
        case WIDTH_DOWN:
            w_down_ = -value;
            break;
        case WIDTH_UP:
            w_up_ = value;
            break;
        case MIX_DRYWET:
            dry_ = 0.005f * (100 - value);
            wet_ = 0.005f * (100 + value) ;
        case GATE:
            gate_wet_ = value;
            break;
        case GATE_SOURCE:
            gate_dry_ = value;
            break;
        default:
            break;
        }
        p_[index] = value;
    }

    inline int32_t getParameterValue(uint8_t index) const {
        return p_[index];
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
        gate_ = 1;
//        float bas_hez = osc_notehzf(base_note);
//        float note_hz = osc_notehzf(note);
        float note_hz = note2freq(note);
        w0_ = note_hz / base_hz;
        phi_ = delay_pos_ - 4;
        if (phi_ < 0) {
            phi_ += buffer_size_;
        }
    }

    inline void NoteOff(uint8_t note) {
        (uint8_t)note;
        gate_ = 0;
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
    int32_t gate_;
    float input_gain_;
    float w0_;
    float delay_line_[max_buffer_size];
    size_t buffer_size_ = max_buffer_size;
    size_t buffer_mask_ = buffer_size_ - 1;
    float grain_delay_depth_;
    int32_t delay_pos_;
    uint8_t w_down_;
    uint8_t w_up_;
    float dry_;
    float wet_;
    int32_t gate_dry_ = 0;
    int32_t gate_wet_ = 0;

    // calcurate w0 for SHIFT knob
    float shape_w0_value(float p) { // -1.0 <= p <= 1.0
        float n = note_;
        if (p < 0) {
            n += w_down_ * p;
        } else {
            n += w_up_ * p;
        }
        return note2freq(n) / base_hz;
    }

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

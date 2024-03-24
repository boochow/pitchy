#pragma once
/*
 *  File: osc.h
 *
 *  oscillator for Pitchy
 *
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <climits>

#include "unit_osc.h"   // Note: Include base definitions for osc units
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

        return k_unit_err_none;
    }

    inline void Teardown() {
    }

    inline void Reset() {
        phi_ = 0.f;
        gate_ = 0;
        input_gain_ = 6.89f;
        w0_ = 1.f;
    }

    inline void Resume() {
    }

    inline void Suspend() {
    }

    fast_inline void Process(const float * in, float * out, size_t frames) {
        const float * __restrict in_p = in;
        float * __restrict out_p = out;
        const float * out_e = out_p + frames;  // assuming mono output
        const int32_t gate_wet = (gate_wet_) ? gate_ : 1;
        const int32_t gate_dry = (gate_dry_) ? gate_ : 1;

        for (; out_p != out_e; in_p += 2, out_p += 1) {
            float in_sig = *in_p + *(in_p + 1);

            grain_write(in_sig, grain_delay_depth_);
            float out_sig = grain_read(phi_);
            phi_ += w0_;
            phi_ = ((int) phi_ & buffer_mask_) + (phi_ - (int) phi_);
            out_sig = wet_ * out_sig * gate_wet + dry_ * in_sig * gate_dry;
            *(out_p) = osc_softclipf(0.1f, out_sig * input_gain_);
        }
    }

    inline void setParameter(uint8_t index, const int32_t value) {
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
            if (value != p_[GRAIN_SIZE]) {
                buffer_size_ = 1 << (13 - value);
                buffer_mask_ = buffer_size_ - 1;
                grain_pos_ = 2;
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
            break;
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
        float note_hz = note2freq(note);
        w0_ = note_hz / base_hz;
        phi_ = grain_pos_ - 2;
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

    int32_t p_[10] = {511, 431, 2, 0, -2, 2, 100, 0, 0, 0};
    float phi_ = 0.f;
    uint8_t note_ = base_note;
    int32_t gate_ = 0;
    float input_gain_ = 6.89f;
    float w0_ = 1.f;
    float grain_buf_[max_buffer_size];
    size_t buffer_size_ = 1 << (13 - p_[2]);
    size_t buffer_mask_ = buffer_size_ - 1;
    float grain_delay_depth_ = 0.f;
    int32_t grain_pos_ = 2;
    uint8_t w_down_ = 2;
    uint8_t w_up_ = 2;
    float dry_ = 0.f;
    float wet_ = 1.f;
    int32_t gate_dry_ = 0;
    int32_t gate_wet_ = 0;

    // calcurate w0 for the SHAPE knob
    float shape_w0_value(float p) { // -1.0 <= p <= 1.0
        float n = note_;
        if (p < 0) {
            n += w_down_ * p;
        } else {
            n += w_up_ * p;
        }
        return note2freq(n) / base_hz;
    }

    void grain_write(const float sig, const float mix) {
        grain_buf_[grain_pos_++] = sig + mix * grain_buf_[grain_pos_];
	grain_pos_ = grain_pos_ & buffer_mask_;
    }

    float grain_read(float pos) {
	int32_t p0 = int(pos);
        float frac = pos - p0;
        int32_t p1 = (p0 + 1) & buffer_mask_;
	return linintf(frac, grain_buf_[p0], grain_buf_[p1]);
    }
};

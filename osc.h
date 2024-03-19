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
constexpr size_t buffer_size = 1 << 12;
constexpr size_t buffer_mask = buffer_size - 1;

inline float note2freq(float note) {
    return (440.f / 32) * powf(2, (note - 9.0) / 12);
}

class Osc {
public:
    enum {
        SHAPE = 0U,
        ALT,
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
        vol_ = 128.f;
        w0_ = 1.0f;

        return k_unit_err_none;
    }

    inline void Teardown() {
    }

    inline void Reset() {
        phi_ = 0;
        note_ = 0;
        vol_ = 128.f;
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
            delay_write(in_p);
            float sig = delay_read(phi_) + delay_read(phi_ + 1);
            phi_ += 2 * w0_;
            if (((int) phi_) >= buffer_size) {
                phi_ -= buffer_size;
            }
//            *(out_p) = sig * vol_ + (256. - vol_) * (*in_p + *(in_p + 1));
            *(out_p) = sig * vol_;
        }
    }

    inline void setParameter(uint8_t index, int32_t value) {
        switch(index) {
        case SHAPE:
            w0_ = value * 0.001 + 0.75f;
            break;
        case ALT:
            vol_ = 0.25f * value;
            break;
        default:
            break;
        }
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
            phi_ += buffer_size;
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

    float phi_;
    uint8_t note_;
    bool gate_;
    float vol_;
    float w0_;
    float delay_line_[buffer_size];
    int32_t delay_pos_;

    void delay_write(const float *sig) {
        delay_line_[delay_pos_++] = *sig;
        delay_line_[delay_pos_++] = *(sig + 1);
	delay_pos_ = delay_pos_ & buffer_mask;
    }

    float delay_read(float pos) {
	// Note: this code doesn't interpolate frac part of pos
	int32_t p = int(pos);
	p = (delay_pos_ - (p << 1)) & buffer_mask;
	return delay_line_[p];
    }
};

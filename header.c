/*
    BSD 3-Clause License

    Copyright (c) 2023, KORG INC.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//*/

/*
 *  File: header.c
 *
 *  NTS-1 mkII oscillator unit header definition
 *
 */

#include "unit_osc.h"   // Note: Include base definitions for osc units

// ---- Unit header definition  --------------------------------------------------------------------

const __unit_header unit_header_t unit_header = {
    .header_size = sizeof(unit_header_t),                  // Size of this header. Leave as is.
    .target = UNIT_TARGET_PLATFORM | k_unit_module_osc,    // Tagret platform and module pair for this unit
    .api = UNIT_API_VERSION,                               // API version for which unit was built. See runtime.h
    .dev_id = 0x42636877U,  // "Bchw"
    .unit_id = 0x01010000,  // Product number(01),Unit type(01=Synth),reserved
    .version = 0x00010000U,
    .name = "PSHFTR",
    .num_params = 9,
    .params = {
        // Format:
        // min, max, center, default, type, frac. bits, frac. mode, <reserved>, name
        // Fixed/direct UI parameters
        // A knob
        {0, 1023, 0, 0, k_unit_param_type_none, 0, 0, 0, {"SHPE"}},

        // B knob
        {0, 1023, 511, 431, k_unit_param_type_none, 0, 0, 0, {"ALT"}},

        // 8 Edit menu parameters
        {0, 8, 0, 0, k_unit_param_type_none, 0, 0, 0, {"SIZE"}},
        {0, 100, 50, 0, k_unit_param_type_percent, 0, 0, 0, {"DPTH"}},
        {-24, 0, -12, -2, k_unit_param_type_none, 0, 0, 0, {"DOWN"}},
        {0, 24, 12, 2, k_unit_param_type_none, 0, 0, 0, {"UP"}},
        {-100, 100, 0, 100, k_unit_param_type_drywet, 0, 0, 0, {"MIX"}},
        {0, 1, 0, 0, k_unit_param_type_onoff, 0, 0, 0, {"GT-O"}},
        {0, 1, 0, 0, k_unit_param_type_onoff, 0, 0, 0, {"GT-I"}},
        {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}}},
};

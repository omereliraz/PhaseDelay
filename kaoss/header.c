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
 *  NTS-3 kaoss pad kit generic effect unit header definition
 *
 */

#include "unit_genericfx.h"   // Note: Include base definitions for genericfx units

// ---- Unit header definition  --------------------------------------------------------------------

const __unit_header genericfx_unit_header_t unit_header = {
  .common = {
    .header_size = sizeof(genericfx_unit_header_t),           // Size of this header. Leave as is.
    .target = UNIT_TARGET_PLATFORM | k_unit_module_genericfx, // Target platform and module pair for this unit
    .api = UNIT_API_VERSION,                                  // API version for which unit was built. See runtime.h
    //.dev_id = 0x4B4F5247, //KORG                              // Developer ID. See https://github.com/korginc/logue-sdk/blob/master/developer_ids.md
    .dev_id = 0x4c504f50,
    .unit_id = 0x1U,                                          // ID for this unit. Scoped within the context of a given dev_id.
    .version = 0x00010006U,                                   // This unit's version: major.minor.patch (major<<16 minor<<8 patch).
    .name = "Phase Loopop",                                          // Name for this unit, will be displayed on device
    .num_params = 4,                                          // Number of valid parameter descriptors. (max. 8)
    
    .params = {
      // Format: min, max, center, default, type, frac. bits, frac. mode, <reserved>, name     
      {0, 1023, 0, 0, k_unit_param_type_none, 0, 0, 0, {"TIME"}},
      {0, 1023, 0, 0, k_unit_param_type_none, 0, 0, 0, {"FEEDBACK"}},    
      {0, 1023, 0, 0, k_unit_param_type_none, 0, 0, 0, {"RATIO"}},      
      {0, 1023, 0, 0, k_unit_param_type_none, 0, 0, 0, {"DRYWET"}},
      {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}},
      {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}},
      {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}},
      {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}}},
  },
  .default_mappings = {
    {k_genericfx_param_assign_x,     k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 1023, 512},
    {k_genericfx_param_assign_y,     k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 1023, 512},
    {k_genericfx_param_assign_none,  k_genericfx_curve_linear, k_genericfx_curve_unipolar, 512, 1023, 768},
    {k_genericfx_param_assign_depth, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 512, 512},
    
    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 0, 0},
    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 0, 0},
    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 0, 0},
    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 0, 0}
  }
};

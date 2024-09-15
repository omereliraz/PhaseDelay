#pragma once
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
 *  File: effect.h
 *
 *  Dummy generic effect template instance.
 *
 */

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <climits>

#include "unit_genericfx.h"   // Note: Include base definitions for genericfx units

#include "utils/buffer_ops.h"
#include "utils/int_math.h"

#include "dsp/biquad.hpp"

#define DELAY_LINE_L_SIZE 192000
#define DELAY_LINE_R_SIZE DELAY_LINE_L_SIZE
#define BUFFER_SIZE DELAY_LINE_L_SIZE + DELAY_LINE_R_SIZE

class Effect {
public:
  /*===========================================================================*/
  /* Public Data Structures/Types/Enums. */
  /*===========================================================================*/

  enum {
    L_DELAY_BUF_SIZE = 0x20000,
    R_DELAY_BUF_SIZE = 0x15000,
    APF_BUF_SIZE = 0x4000U,
    REVERB_DSP_BUF_SIZE = L_DELAY_BUF_SIZE + R_DELAY_BUF_SIZE + APF_BUF_SIZE
  };

  enum {
    TIME = 0U,
    FEEDBACK,
    RATIO,
    DRYWET, 
    NUM_PARAMS
  };

  // Note: Make sure that default param values correspond to declarations in header.c
  struct Params {
    float time{0.f};
    float feedback{0.f};
    float ratio{0.f};
    float drywet{0.f};
   

    void reset() {
      time = 0.f;
      feedback = 0.f;
      ratio = 0.f;
      drywet = 0.f;
    }
  }; 

  
  /*===========================================================================*/
  /* Lifecycle Methods. */
  /*===========================================================================*/

  Effect(void) {}
  ~Effect(void) {} // Note: will never actually be called for statically allocated instances

  inline int8_t Init(const unit_runtime_desc_t * desc) {
    if (!desc)
      return k_unit_err_undef;
    
    // Note: make sure the unit is being loaded to the correct platform/module target
    if (desc->target != unit_header.common.target)
      return k_unit_err_target;
    
    // Note: check API compatibility with the one this unit was built against
    if (!UNIT_API_IS_COMPAT(desc->api))
      return k_unit_err_api_version;
    
    // Check compatibility of samplerate with unit, for NTS-3 kaoss pad kit should be 48000
    if (desc->samplerate != 48000)
      return k_unit_err_samplerate;

    // Check compatibility of frame geometry
    if (desc->input_channels != 2 || desc->output_channels != 2)  // should be stereo input/output
      return k_unit_err_geometry;


    // If SDRAM buffers are required they must be allocated here
    if (!desc->hooks.sdram_alloc)
      return k_unit_err_memory;
    float *m = (float *)desc->hooks.sdram_alloc(BUFFER_SIZE*sizeof(float));    
    if (!m)
      return k_unit_err_memory;

    buf_clr_f32(m, BUFFER_SIZE);

    reverb_dsp_buf_ = m;
    ldelay_buf_ = m;
    m += DELAY_LINE_L_SIZE;
    rdelay_buf_ = m;
    //m += DELAY_LINE_R_SIZE;
    //allpassfilter_buf_ = m;

    
    // Cache the runtime descriptor for later use
    runtime_desc_ = *desc;

    // Make sure parameters are reset to default values
    params_.reset();
    
    return k_unit_err_none;
  }

  inline void Teardown() {
    // Note: buffers allocated via sdram_alloc are automatically freed after unit teardown
    // Note: cleanup and release resources if any
    reverb_dsp_buf_ = nullptr;
    ldelay_buf_ = nullptr;
    rdelay_buf_ = nullptr;
    allpassfilter_buf_ = nullptr;
  }

  inline void Reset() {
    // Note: Reset effect state, excluding exposed parameter values.
  }

  inline void Resume() {
    // Note: Effect will resume and exit suspend state. Usually means the synth
    // was selected and the render callback will be called again

    // Note: If it is required to clear large memory buffers, consider setting a flag
    //       and trigger an asynchronous progressive clear on the audio thread (Process() handler)
    s_clear_lines = true;
  }

  inline void Suspend() {
    // Note: Effect will enter suspend state. Usually means another effect was
    // selected and thus the render callback will not be called
  }

  /*===========================================================================*/
  /* Other Public Methods. */
  /*===========================================================================*/

  fast_inline void Process(const float * in, float * out, size_t frames) {
    const float * __restrict in_p = in;
    float * __restrict out_p = out;
    const float * out_e = out_p + (frames << 1);  // assuming stereo output

    // cache variables
    uint32_t delayLineL_Wr = l_writeidx;
    uint32_t delayLineR_Wr = r_writeidx;

    const Params p = params_;
    const float valTime = p.time;
    const float valDepth = p.feedback;
    const float valWet = p.drywet;
    const float valDry = 1.0f - p.drywet;
    const float valRatio = p.ratio;
    
    //for (; out_p != out_e; in_p += 2, out_p += 2) {
    for(uint32_t curFrame = 0; curFrame < frames; curFrame++){
      
      float sigInL = in_p[curFrame * 2];
      float sigInR = in_p[curFrame * 2 + 1];

      float leftLineSize = DELAY_LINE_L_SIZE * valTime;
      float rightLineSize = DELAY_LINE_R_SIZE * valTime * valRatio;

      if (delayLineL_Wr++ >= leftLineSize){
        delayLineL_Wr = 0;
      }
      if (delayLineR_Wr++ >= rightLineSize){
        delayLineR_Wr = 0;
      }
            
      float delayLineSigL = ldelay_buf_[delayLineL_Wr];
      float delayLineSigR = rdelay_buf_[delayLineR_Wr];

      ldelay_buf_[delayLineL_Wr] = (sigInL + delayLineSigL) * valDepth;
      rdelay_buf_[delayLineR_Wr] = (sigInR + delayLineSigR) * valDepth;

      float sigOutL = (sigInL * valDry) + (delayLineSigL * valWet);
      float sigOutR = (sigInR * valDry) + (delayLineSigR * valWet);
 
      out_p[curFrame * 2] = sigOutL;
      out_p[curFrame * 2 + 1] = sigOutR;

    }
    r_writeidx = delayLineR_Wr;
    l_writeidx = delayLineL_Wr;
  }

  inline void setParameter(uint8_t index, int32_t value) {
    switch (index) {
    case TIME: // 10bit 0-1023 parameter      
      value = clipminmaxi32(0, value, 1023);
      params_.time = param_10bit_to_f32(value); // 0 .. 1023 -> 0.0 .. 1.0
      break;
    case FEEDBACK: // 10bit 0-1023 parameter
      value = clipminmaxi32(0, value, 1023);
      params_.feedback = param_10bit_to_f32(value);
      break;
    case RATIO: // 10bit 0-1023 parameter      
      value = clipminmaxi32(0, value, 1023);
      params_.ratio = param_10bit_to_f32(value);
      break;
    case DRYWET:
      value = clipminmaxi32(0, value, 1023);
      params_.drywet = param_10bit_to_f32(value);
      break;      
    default:
      break;
    }
  }

  inline int32_t getParameterValue(uint8_t index) const {
    switch (index) {
    case TIME: // 10bit 0-1023 parameter
      return param_f32_to_10bit(params_.time);
      break;
    case FEEDBACK: // 10bit 0-1023 parameter      
      return param_f32_to_10bit(params_.feedback);
      break;
    case RATIO: // 10bit 0-1023 parameter      
      return param_f32_to_10bit(params_.ratio);
      break;
    case DRYWET: // 10bit 0-1023 parameter      
      return param_f32_to_10bit(params_.drywet);
      break;      
    default:
      break;
    }

    return INT_MIN; // Note: will be handled as invalid
  }

  inline const char * getParameterStrValue(uint8_t index, int32_t value) const {
    // Note: String memory must be accessible even after function returned.
    //       It can be assumed that caller will have copied or used the string
    //       before the next call to getParameterStrValue    
    return nullptr;
  }
  
  inline void setTempo(uint32_t tempo) {
    // const float bpmf = (tempo >> 16) + (tempo & 0xFFFF) / static_cast<float>(0x10000);
    (void)tempo;
  }

  inline void tempo4ppqnTick(uint32_t counter) {
    (void)counter;
  }

  inline void touchEvent(uint8_t id, uint8_t phase, uint32_t x, uint32_t y) {    
    (void)id;
    (void)phase;
    (void)x;
    (void)y;
  }
  
  /*===========================================================================*/
  /* Static Members. */
  /*===========================================================================*/
  
private:
  /*===========================================================================*/
  /* Private Member Variables. */
  /*===========================================================================*/

  std::atomic_uint_fast32_t flags_;
  unit_runtime_desc_t runtime_desc_;
  Params params_;

  uint32_t s_cleared_frames;
  bool     s_clear_lines;
  
  float *reverb_dsp_buf_;
  float *ldelay_buf_;  
  float *rdelay_buf_;
  float *allpassfilter_buf_;

  uint32_t s_writeidx = 0;
  uint32_t l_writeidx = 0;
  uint32_t r_writeidx = 0;
  float    s_drymixz = 1.f;
  float    s_wetmixz = 1.f;  
  float    timeratio = 0.f;

  float s_peak_holdz = 0.f;
  float s_gatez = 1.f;
  float s_rmsz = 1.f;

  float apf1234In[4] __attribute__((aligned(4)));
  float combhidamp[4] __attribute__((aligned(4)));
  float combhidampz[4] __attribute__((aligned(4))); 

  
  /*===========================================================================*/
  /* Private Methods. */
  /*===========================================================================*/

  void updateHighDamp(float damp){
    damp = (damp * 0.5f) + 0.5f;
    const float comb1 = damp * 1.0f;
    const float comb2 = damp * 0.9f;
    const float comb3 = damp * 0.85f;
    const float comb4 = damp * 0.77f;
    combhidamp[0] = (comb1);
    combhidamp[1] = (comb2);
    combhidamp[2] = (comb3);
    combhidamp[3] = (comb4);    
  }

  /*===========================================================================*/
  /* Constants. */
  /*===========================================================================*/

  const float interp_ratio_48KHz = 0.002083333333333f; // ~480 samples @ 48KHz -> ~10ms 

  const float PeakHoldCoeff = 0.999725f;
  const float gate_attack_time = 0.075f;
  const float gate_release_time = 0.00003f;

  const uint32_t s_apf1234_time[4] __attribute__((aligned(4))) = {
    0x0570,
    0x0a30,
    0x0740,
    0x0470
  };

  const uint32_t s_comb_fbtime[4] __attribute__((aligned(4))) = {
    0x1ba0,
    0x19e0,
    0x1740,
    0x1590
  };

  const uint32_t s_comb_tapltime[4] __attribute__((aligned(4))) = {
    0x0ae0,
    0x0a50,
    0x0780,
    0x04b0
  };

  const uint32_t s_comb_taprtime[4] __attribute__((aligned(4))) = {
    0x0e10,
    0x0690,
    0x05a0,
    0x02e0
  };
};

#include "userdelfx.h"
#include "float_math.h"
#include "buffer_ops.h"


#define DELAY_LINE_L_SIZE 96000   // size of delay line
#define DELAY_LINE_R_SIZE (int) (DELAY_LINE_L_SIZE * 0.85)   // size of delay line
#define SAMPLE_RATE 48000    // 48KHz is our fixed sample rate

// Delay BPM division with time knob from 0 to full:
// 1/64, 1/48, 1/32, 1/24, 1/16, 1/12, 1/8, 1/6, 3/16, 1/4, 1/3, 3/8, 1/2, 3/4, 1
//float delayDivisions[NUM_DELAY_DIVISIONS] =
//    {0.015625,.02083333,.03125,.04166666,.0625f,.08333333f,.125f,
//     .16666667f,.1875f,.25f,.33333333f,.375f,.5f,.75f,1};



// delay lines
__sdram float delayLine_L[DELAY_LINE_L_SIZE];
__sdram float delayLine_R[DELAY_LINE_R_SIZE];

// Current position in the delay line we are writing to:
// (integer value as it is per-sample)
uint32_t delayLineL_Wr = 0;
uint32_t delayLineR_Wr = 0;


// Smoothing (glide) for delay time:
// This is the current delay time as we smooth it
float currentDelayTime = 48000;

// Depth knob value from 0-1
float valDepth = 0.5f;

// Time value knob from 0-1
float valTime = 0.5f;

// Wet/Dry signal levels
float wet = 0.5f;
float dry = 0.5f;


void DELFX_INIT(uint32_t platform, uint32_t api)
{

  // Initialize the variables used
  delayLineL_Wr = 0;
  delayLineR_Wr = 0;

  // Clear the delay lines.
  for (int i=0;i<DELAY_LINE_L_SIZE;i++)
    {
      delayLine_L[i] = 0;
    }
  for (int i=0;i<DELAY_LINE_R_SIZE;i++)
    {
      delayLine_R[i] = 0;
    }


  valDepth = 0.5f;
  valTime = 0.5f;

  wet = 0.5f;
  dry = 0.5f;
}

void DELFX_PROCESS(float *xn, uint32_t frames)
{
  float * __restrict x = xn; // Local pointer, pointer xn copied here.
  const float * x_e = x + 2 * frames; // End of data buffer address

  for(uint32_t curFrame = 0; curFrame < frames; curFrame++){

      // read input signal
      float sigInL = xn[curFrame * 2]; // get the value pointed at x (Left channel)
      float sigInR = xn[curFrame * 2 + 1]; // get the value pointed at x + 1 (right channel)

      // read delay line signal
      float leftLineSize = DELAY_LINE_L_SIZE * valTime;
      float rightLineSize = DELAY_LINE_R_SIZE * valTime;
      if(delayLineL_Wr++ >= leftLineSize) delayLineL_Wr = 0;
      if(delayLineR_Wr++ >= rightLineSize) delayLineR_Wr = 0;
      float delayLineSig_L = delayLine_L[delayLineL_Wr];
      float delayLineSig_R = delayLine_R[delayLineR_Wr];


      // update delay line
      delayLine_L[delayLineL_Wr] = (sigInL + delayLineSig_L) * valDepth;
      delayLine_R[delayLineR_Wr] = (sigInR + delayLineSig_R) * valDepth;


      // Generate our output signal:
      float sigOutL = sigInL * dry + delayLineSig_L * wet;
      float sigOutR = sigInR * dry + delayLineSig_R * wet;

      // update output signal
      xn[curFrame * 2] = sigOutL;
      xn[curFrame * 2 + 1] = sigOutR;

    }



}

void DELFX_PARAM(uint8_t index, int32_t value)
{
  const float valf = q31_to_f32(value);
  float s_mix; // Used for wet/dry calculations
  switch (index) {
      case k_user_delfx_param_time:
        valTime = valf;
      break;
      case k_user_delfx_param_depth:
        valDepth = valf;
      break;
      case k_user_delfx_param_shift_depth:
        s_mix = (valf <= 0.49f) ? 1.02040816326530612244f * valf : (valf >= 0.51f) ? 0.5f + 1.02f * (valf-0.51f) : 0.5f;
        wet = s_mix;
        dry = 1.0f - s_mix;
      break;
      default:
        break;
    }
}

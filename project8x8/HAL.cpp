#include "HAL.h"


hw_timer_t* timer_0 = NULL;
hw_timer_t* timer_1 = NULL;
hw_timer_t* timer_2 = NULL;
hw_timer_t* timer_3 = NULL; // reserved for ADC

void blockInterrupts()
{
  xt_ints_off(~(0b1 << 6));
}

void unblockInterrupts()
{
  xt_ints_on(0xFFFFFFFF);
}

void setupPwm(uint8_t chan, double freq)
{
    if(freq < 1 || 312500 < freq || chan > 15)  return;
    ledcSetup(chan, (double) freq, 8);
}

void setPwmDuty(uint8_t chan, uint16_t duty)
{
    if(chan > 15)  return;
    ledcWrite(chan, duty);
}

void pwmAttachPin(uint8_t chan, uint8_t pin)
{
    if(pin > 39)  return;
    ledcAttachPin(pin, chan);
}

void pwmDetachPin(uint8_t pin)
{
    if(pin > 39)  return;
    ledcDetachPin(pin);
}

void setUpTimer(uint8_t timer_index, void (*f)(), uint64_t period) {

    switch(timer_index){
      case 0:
        timer_0 = timerBegin(timer_index, 80, true);
        timerAttachInterrupt(timer_0, f, true);
        timerAlarmWrite(timer_0, period, true);
        break;
      case 1:
        timer_1 = timerBegin(timer_index, 80, true);
        timerAttachInterrupt(timer_1, f, true);
        timerAlarmWrite(timer_1, period, true);
        break;
      case 2:
        timer_2 = timerBegin(timer_index, 80, true);
        timerAttachInterrupt(timer_2, f, true);
        timerAlarmWrite(timer_2, period, true);
        break;
      case 3:
        timer_3 = timerBegin(timer_index, 80, true);
        timerAttachInterrupt(timer_3, f, true);
        timerAlarmWrite(timer_3, period, true);
        break;
      default:
        break;
    }
}

void startTimer(uint8_t timer_index)
{
  switch(timer_index){
      case 0:
        timerAlarmEnable(timer_0);
        break;
      case 1:
        timerAlarmEnable(timer_1);
        break;
      case 2:
        timerAlarmEnable(timer_2);
        break;
      case 3:
        timerAlarmEnable(timer_3);
        break;
      default:
        break;
    }
}

void stopTimer(uint8_t timer_index) {
    switch(timer_index){
      case 0:
        timerEnd(timer_0);
        break;
      case 1:
        timerEnd(timer_1);
        break;
      case 2:
        timerEnd(timer_2);
        break;
      case 3:
        timerEnd(timer_3);
        break;
      default:
        break;
    }

}

void changeTimerPeriod(uint8_t timer_index, uint64_t period) {
  switch(timer_index){
      case 0:
        timerAlarmWrite(timer_0, period, true);
        break;
      case 1:
        timerAlarmWrite(timer_1, period, true);
        break;
      case 2:
        timerAlarmWrite(timer_2, period, true);
        break;
      case 3:
        timerAlarmWrite(timer_3, period, true);
        break;
      default:
        break;
    }
}

void setUpDAC(uint8_t channel) {
  if (channel<1 || channel>2) return;
  dac_output_enable((dac_channel_t) channel);
}

void writeToDAC(uint8_t channel, uint8_t value) {
  if (channel<1 || channel>2) return;
  dac_output_voltage((dac_channel_t) channel, value);
}



///*
// * Function:  dacSineGeneratorStart
// * --------------------
// *  Generates a Sine wave of the desired frequency on the indicated channel with an option to invert it
// * 
// *  Note that the two channels share the same frequncy, the frequency of the generated sine wave will
// *  be the one indicated in the most recent call of dacSineGeneratorStart()
// *
// *  channel:        indicates which of the two dac outputs (25 or 26) we want to use
// *                  valid values are 25 or 26 (this corresponds to GPIO25 or GPIO26)
// *                  
// *  inv:            inverts the wave if true;
// *
// *  freq:           sets the frequency. The unit is in multiples of 120 hertz.
// *                  So for a frequency of 120 kHz, set freq to 1000
// * 
// * returns:        void
// */
//void dacSineGeneratorStart(uint8_t channel, bool inv, uint16_t freq)
//{
//  if(channel < 25 || channel > 26) return;
//  SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
//  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, freq, SENS_SW_FSTEP_S);
//  if(channel)
//  {
//    SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
//    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV2, 0b10 + inv, SENS_DAC_INV2_S);
//    dac_output_enable(DAC_CHANNEL_2);
//  }
//  else
//  {
//    SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
//    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, 0b10 + inv, SENS_DAC_INV1_S);
//    dac_output_enable(DAC_CHANNEL_1);
//  }
//}
//
//
///*
// * Function:  dacSineGeneratorSetFrequency
// * --------------------
// * Sets the frequency of the sine wave generator
// * 
// *  freq:           sets the frequency. The unit is in multiples of 120 hertz.
// *                  So for a frequency of 120 kHz, set freq to 1000
// * 
// * returns:        void
// */
//void dacSineGeneratorSetFrequency(uint16_t freq)
//{
//  SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, freq, SENS_SW_FSTEP_S);
//}
//
//
///*
// * Function:  dacSineGeneratorStop
// * --------------------
// * Stops the sine wave generator
// * 
// * returns:        void
// */
//void dacSineGeneratorStop(uint8_t channel)
//{
//  channel = channel - 25;
//  if(channel > 1) return;
//  if(channel)
//  {
//    dac_output_disable(DAC_CHANNEL_2);
//    return;
//  }
//  dac_output_disable(DAC_CHANNEL_1);
//}

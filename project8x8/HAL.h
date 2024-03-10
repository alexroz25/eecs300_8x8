#ifndef HAL_H
#define HAL_H


#include <esp32-hal-ledc.h>
#include <driver/adc.h>
#include <driver/dac.h>
//#include <stdio.h>
#include <Wire.h>
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define PUSH_BUTTON_PIN 0
#define LED_PIN 2


/*
 * Function:  blockInterrupts()
 * --------------------
 * temporarily blocks all interrupts from firing
 * interrupts can be unblocked with unblockInterrupts()
 *
 *  returns: void
 */
void blockInterrupts();

/*
 * Function:  unblockInterrupts()
 * --------------------
 * unblocks enabled interrupts from firing
 *
 * returns: void
 */
void unblockInterrupts();

/*
 * Function:  setupPwm
 * --------------------
 * configures a channel to output a PWM signal at a desired frequency
 * and starts the pwm generation
 * note that you must call pwmAttachPin() in order to actually output a signal
 * to a physical pin
 * More info: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html
 *
 *  chan:   Specifies which of the 16 available channels we want to setup (range 0 - 15)
 *          
 *  freq:   Specifies the PWM frequency in hertz (range 4 - 312500). 
 *          Since each period of the PWM signal is broken into 256 parts (8 bit counter), 
 *          the "counting frequency" [Hz] will be freq*256.
 *
 *  returns: void
 */
void setupPwm(uint8_t chan, double freq);

/*
 * Function:  setPwmDuty
 * --------------------
 *  sets the duty cycle of a given pwm channel
 *
 *  chan:   Specifies which of the 16 available channels we want to setup (range 0 - 15)
 *
 *  duty:   Specifies the duty cycle, where 0 is 0% and 255 is 100% (range 0 - 255)
 *
 *  returns: void
 */
void setPwmDuty(uint8_t chan, uint16_t duty);

/*
 * Function:  pwmAttachPin
 * --------------------
 *  configures a pin to output the indicated pwm channel
 *  
 *  chan:     Specifies which of the 16 available channels we want to setup (range 0 - 15)
 *
 *  pin:      Specifies the pin we wish to output the indicated channel to
 *            This corresponds to the GPIO pin number (range 0 - 39)
 *
 *  returns:  void
 */
void pwmAttachPin(uint8_t chan, uint8_t pin);

/*
 * Function:  pwmDetachPin
 * --------------------
 *  detaches the indicate pin form the pwm channel
 *
 *  pin:      Specifies the pin we wish to output the indicated channel to
 *            This corresponds to the GPIO pin number (range 0 - 39)
 *
 *  returns:  void
 */
void pwmDetachPin(uint8_t pin);

/*
 * Function:  setUpTimer
 * --------------------
 *  configures a timed interrupt with the desired period and callback function
 *  the counting frequency is 1 MHz
 *
 *  timer_index:    indicates which of the four available interrupt generation timers 
 *                  we want to use. (range 0 - 3)
 *  
 *  void (*f)():    a function pointer (which points to the desired callback function)
 *                  to use this, simply write the name of the callback function
 *
 *  period:         indicates the desired period (in microseconds) of the timed interrupt  
 *
 *  returns:        void
 */
void setUpTimer(uint8_t timer_index, void (*f)(), uint64_t period);

/*
 * Function:  startTimer
 * --------------------
 *  starts the indicated interrupt generation timer
 *
 *  timer_index:    indicates which of the four available interrupt generation timers 
 *                  we want to use. (range 0 - 3)
 *
 *  returns:        void
 */
void startTimer(uint8_t timer_index);

/*
 * Function:  stopTimer
 * --------------------
 *  stops the indicated interrupt generation timer
 *
 *  timer_index:    indicates which of the four available interrupt generation timers 
 *                  we want to use. (range 0 - 3)
 *
 *  returns:        void
 */
void stopTimer(uint8_t timer_index);


/*
 * Function:  changeTimerPeriod
 * --------------------
 *  changes the period of the indicated interrupt generation timer to the desired value
 *
 *  timer_index:    indicates which of the four available interrupt generation timers 
 *                  we want to use. (range 0 - 3)
 *
 *  period:         indicates the desired period (in microseconds) of the timed interrupt  
 *
 *  returns:        void
 */
void changeTimerPeriod(uint8_t timer_index, uint64_t period);


/*
 * Function:  setUpDAC
 * --------------------
 *  Initializes the desired DAC.
 *
 *  channel:    indicates which of the 2 DACs we want to initialize (range 1 - 2)
 *
 *  returns: void
 */
void setUpDAC(uint8_t channel);

/*
 * Function:  writeToDAC
 * --------------------
 *  Writes a Voltage for the indicated DAC to output
 *
 *  channel:    indicates which of the 2 DACs we want to write to (range 1 - 2)
 *  
 *  value:      determines voltage to output, where 0 = 0V and 255 = 3.3V 
 *
 *  returns:    void
 */
void writeToDAC(uint8_t channel, uint8_t value);


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
//void dacSineGeneratorSetFrequency(uint16_t freq);
//
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
//void dacSineGeneratorStart(uint8_t channel, bool inv, uint16_t freq);
//
//
///*
// * Function:  dacSineGeneratorStop
// * --------------------
// * Stops the sine wave generator
// * 
// * returns:        void
// */
//void dacSineGeneratorStop(uint8_t channel);

#endif

/*
 * bldc_driver_functions.h
 *
 *  Created on: Nov 29, 2017
 *      Author: simon
 */

#ifndef INC_BLDC_DRIVER_FUNCTIONS_H_
#define INC_BLDC_DRIVER_FUNCTIONS_H_

/** Changes the output channels for the pwm.
    Input:
    state = 0: A heavyside, C lowside
    state = 1: B heavyside, C lowside
    state = 2: B heavyside, A lowside
    state = 3: C heavyside, A lowside
    state = 4: C heavyside, B lowside
    state = 5: A heavyside, B lowside
    state > 5: power off all channels
**/
void changePhaseState(uint8_t state);

/** Sets the dutycycle of the pwm.
    Input:
    0 <= dutyCycle <= 100 every handed value > 100 will be interpreted as 100
 **/
void setPhaseDutyCycle(uint8_t dutyCycle);

/** Returns the active state.
 *  Return:
    state = 0: A heavyside, C lowside
    state = 1: B heavyside, C lowside
    state = 2: B heavyside, A lowside
    state = 3: C heavyside, A lowside
    state = 4: C heavyside, B lowside
    state = 5: A heavyside, B lowside
    state > 5: power off all channels
 */
uint8_t getPhaseState();

#endif /* INC_BLDC_DRIVER_FUNCTIONS_H_ */

#include "bldc_driver_functions.h"
#include "drive_state.h"
#include "bufferedLogger.h"

#define GRADIENT_PERCENT_SPEED_UP ((uint8_t)2)
#define GRADIENT_DIVIDER 128
//#define TIME_60DEG_SPEED_UP_START ((uint16_t)11000)
//#define TIME_60DEG_SPEED_UP_START ((uint16_t)8000)
#define TIME_60DEG_SPEED_UP_START 10000
//#define TIME_60DEG_SPEED_UP_END 2040
#define TIME_60DEG_SPEED_UP_END 1500
#define STEPSIZE 10

static uint32_t time60deg;
static uint32_t timestamp_start;
static uint8_t isActive;

void (*callback)(uint32_t);

// function declarations
void startStartUp(void (*startupFinishedCallback)(uint32_t)) {
	isActive = 1;
	callback = startupFinishedCallback;

	timestamp_start = getTimestamp();
	time60deg = TIME_60DEG_SPEED_UP_START; //us

	setSinusApproximation60DegTime(time60deg);
	control3PhaseSinusApproximation(START_SIN_APPROX_FORWARD);
}

void proceedStartUp() {
	uint32_t deltaTime = calculateDeltaTime(timestamp_start);
	/*time60deg = (TIME_60DEG_SPEED_UP_START
			- deltaTime * GRADIENT_PERCENT_SPEED_UP / 100);*/
	time60deg = (TIME_60DEG_SPEED_UP_START
				- deltaTime/GRADIENT_DIVIDER);
	setSinusApproximation60DegTime(time60deg);

	log_time60Deg(time60deg);

	if (time60deg < TIME_60DEG_SPEED_UP_END || !isActive) {
		callback(time60deg);
	}
}

void stopStartUp() {
	isActive = 0;
}

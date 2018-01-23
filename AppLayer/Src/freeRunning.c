#include "drive_state.h"

// functions
void (*freeRunningStopedCallback)(void);

// variables
static uint8_t isActive;

void startFreeRunning(void (*newFreeRunningStopedCallback)(void))
{
    freeRunningStopedCallback = newFreeRunningStopedCallback;
    isActive = 1;
}

void stopFreeRunning(void)
{
    isActive = 0;
    freeRunningStopedCallback();
}


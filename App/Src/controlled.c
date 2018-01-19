#include "drive_state.h"
#include "bldc_driver_functions.h"

// =============== Defines ===============================================
#define MEASURE

#define TIMING 20
//#define MAX_T_60_DEG 2500
#define MAX_T_60_DEG 5000

// _controller options____________________________________________________
#define P_DIVIDER 128
//#define P_DIVIDER 200
//#define P_DIVIDER 128

#define I_CONTROLLER
#define I_DIVIDER 65536

//#define D_CONTROLLER
#define D_DIVIDER 16

// =============== Variables =============================================
static uint32_t time60deg;

// =============== Function pointers =====================================
static void (*tooSlow_stopped_callback)(uint32_t lastTime60deg);

// =============== Function declarations =================================

// =============== Functions =============================================
void startControlled(uint32_t time60deg_init, void (*tooSlowCallback)(void)) {
	time60deg = time60deg_init;
	tooSlow_stopped_callback = tooSlowCallback;
}

void stopControlled() {
	control3PhaseSinusApproximation(STOP_SIN_APPROX);
}

void informRotorPos_controlled(uint32_t rotorpos) {
	uint32_t targetTime;
	targetTime = (time60deg * (30 - TIMING)) / 60;

	int32_t fault = (targetTime - (time60deg - rotorpos));

	int32_t controllerOut = (fault/P_DIVIDER);

#ifdef I_CONTROLLER
	// integrator active
	static int32_t fault_I += fault;
	controllerOut += fault_I/I_DIVIDER;
#endif
#ifdef D_CONTROLLER
	// differentiator active
	static int32_t last_fault;
	controllerOut += (fault-last_fault)/D_DIVIDER;
#endif

	time60deg = time60deg - controllerOut;

	setSinusApproximation60DegTime(time60deg);

	if (time60deg > MAX_T_60_DEG) {
		// too slow
		control3PhaseSinusApproximation(STOP_SIN_APPROX);
		tooSlow_stopped_callback(time60deg);
	}
}

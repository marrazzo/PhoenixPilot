/**
 ******************************************************************************
 *
 * @file       multirotorpathfollower.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      This module compared @ref PositionActuatl to @ref ActiveWaypoint 
 * and sets @ref AttitudeDesired.  It only does this when the FlightMode field
 * of @ref ManualControlCommand is Auto.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/**
 * Input object: ActiveWaypoint
 * Input object: PositionActual
 * Input object: ManualControlCommand
 * Output object: AttitudeDesired
 *
 * This module will periodically update the value of the AttitudeDesired object.
 *
 * The module executes in its own thread in this example.
 *
 * Modules have no API, all communication to other modules is done through UAVObjects.
 * However modules may use the API exposed by shared libraries.
 * See the OpenPilot wiki for more details.
 * http://www.openpilot.org/OpenPilot_Application_Architecture
 *
 */

#include "openpilot.h"
#include "CoordinateConversions.h"

#include "attitudeactual.h"
#include "flightstatus.h"
#include "hwsettings.h"
#include "multirotorpathfollowersettingscc.h"
#include "pathdesired.h"
#include "positionactual.h"
#include "stabilizationdesired.h"
#include "velocityactual.h"

// Private constants
#define MAX_QUEUE_SIZE 4
#define STACK_SIZE_BYTES 1548
#define TASK_PRIORITY (tskIDLE_PRIORITY+2)
#define F_PI 3.14159265358979323846f
#define RAD2DEG (180.0f/F_PI)
#define DEG2RAD (F_PI/180.0f)
#define GRAV 9.805f
// Private types
static struct Integral {
	
	float position_error_North;
	float position_error_East;
	float position_error_Down;
	
} *integral;

// Private variables
static bool followerEnabled = false;
static xTaskHandle multirotorPathFollowerTaskHandle;
static uint8_t flightMode=FLIGHTSTATUS_FLIGHTMODE_MANUAL;
static bool flightStatusUpdate = false;

// Private functions
static void multirotorPathFollowerTask(void *parameters);
static void FlightStatusUpdatedCb(UAVObjEvent * ev);
static void updateMultiRotorDesiredAttitude(MultiRotorPathFollowerSettingsCCData multirotorpathfollowerSettings);
static float bound(float val, float min, float max);

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t MultiRotorPathFollowerStart()
{
	if (followerEnabled) {
		
		// Allocate memory to integral structure
		integral = (struct Integral *)pvPortMalloc(sizeof(struct Integral));
		memset(integral, 0, sizeof(struct Integral));
		
		// Start main task
		xTaskCreate(multirotorPathFollowerTask, (signed char *)"MultiRotorPathFollower", STACK_SIZE_BYTES/4, NULL, TASK_PRIORITY, &multirotorPathFollowerTaskHandle);
		TaskMonitorAdd(TASKINFO_RUNNING_MULTIROTORPATHFOLLOWER, multirotorPathFollowerTaskHandle);
	}

	return 0;
}

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t MultiRotorPathFollowerInitialize()
{
	// Initialize HwSettings UAVO
	HwSettingsInitialize();
	
	// Get the Modules vector from the HwSettings UAVO
	uint8_t optionalModules[HWSETTINGS_OPTIONALMODULES_NUMELEM];
	HwSettingsOptionalModulesGet(optionalModules);
	
	// This is an optional module, so check if it has been enabled.
	if (optionalModules[HWSETTINGS_OPTIONALMODULES_MULTIROTORPATHFOLLOWER] == HWSETTINGS_OPTIONALMODULES_ENABLED) {
		followerEnabled = true;
		MultiRotorPathFollowerSettingsCCInitialize();
		PathDesiredInitialize();
		PositionActualInitialize();
		VelocityActualInitialize();
		StabilizationDesiredInitialize();
		
		FlightStatusConnectCallback(FlightStatusUpdatedCb);
	} else {
		// The module was not enabled.
		followerEnabled = false;
	}
	return 0;
}

MODULE_INITCALL(MultiRotorPathFollowerInitialize, MultiRotorPathFollowerStart)


/**
 * Module thread, should not return.
 */
static void multirotorPathFollowerTask(void *parameters)
{
	portTickType lastUpdateTime;
	
	
	// Main task loop
	lastUpdateTime = xTaskGetTickCount();
	while (1) {
		// Get MultiRotorPathFollowerSettingsCC UAVO
		MultiRotorPathFollowerSettingsCCData multirotorpathfollowerSettings;
		MultiRotorPathFollowerSettingsCCGet(&multirotorpathfollowerSettings);
		
		// Wait until required amount of time has passed
		vTaskDelayUntil(&lastUpdateTime, multirotorpathfollowerSettings.UpdatePeriod / portTICK_RATE_MS);
		
		// Check flightmode
		if (flightStatusUpdate) {
			FlightStatusFlightModeGet(&flightMode);
		}
		
		// Check the flightmode
		switch(flightMode) {
			case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
			case FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME:
				updateMultiRotorDesiredAttitude(multirotorpathfollowerSettings);
				break;
			default:
				// Reset integrals
				memset(integral, 0, sizeof(struct Integral));

				break;
		}
	}
}


/**
 * Compute desired attitude from the desired velocity
 *
 * Takes in @ref NedActual which has the acceleration in the 
 * NED frame as the feedback term and then compares the 
 * @ref VelocityActual against the @ref VelocityDesired
 */
static void updateMultiRotorDesiredAttitude(MultiRotorPathFollowerSettingsCCData multirotorpathfollowerSettings)
{
	//Convert from [ms] to [s]
	float dT = multirotorpathfollowerSettings.UpdatePeriod / 1000.0f;        
	
	// Load state UAVOs
	PositionActualData positionActual;
	PositionActualGet(&positionActual);
	
	VelocityActualData velocityActual;
	VelocityActualGet(&velocityActual);    

	float attitudeActual_Yaw;
	AttitudeActualYawGet(&attitudeActual_Yaw);
	
	// Load control UAVO
	StabilizationDesiredData stabDesired;
	StabilizationDesiredGet(&stabDesired);

	// Load setpoint UAVO
	PathDesiredData pathDesired;
	PathDesiredGet(&pathDesired);
	
	if (flightStatusUpdate) {
		if (flightMode == FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME) {
			// Simple Return To Home mode: fly to home position, (0,0)
			
			pathDesired.End[PATHDESIRED_END_NORTH] = 0;
			pathDesired.End[PATHDESIRED_END_EAST] = 0;
			pathDesired.End[PATHDESIRED_END_DOWN] = positionActual.Down ;
			
		} else if (flightMode == FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD){
			// Simple position hold: stay at present altitude and position
			
			pathDesired.End[PATHDESIRED_END_NORTH]= positionActual.North;
			pathDesired.End[PATHDESIRED_END_EAST] = positionActual.East;
			pathDesired.End[PATHDESIRED_END_DOWN] = positionActual.Down;
		}
		
		//Update UAVO with new values
		PathDesiredSet(&pathDesired);
		
		//Reset integrals
		integral->position_error_North = 0;
		integral->position_error_East = 0;
		integral->position_error_Down= 0;
		
		flightStatusUpdate = false; 
	}
	
	//Compute position error
	float position_error_North= pathDesired.End[0] - positionActual.North;
	float position_error_East = pathDesired.End[1] - positionActual.East;
	float position_error_Down = pathDesired.End[2] - positionActual.Down;
	
	//Compute velocity error
	float velocity_error_North=-velocityActual.North;
	float velocity_error_East =-velocityActual.East;
	float velocity_error_Down =-velocityActual.Down;
	
	/**
	 * Compute desired North acceleration
	 */
	
	float North_Kp = multirotorpathfollowerSettings.NorthPID[MULTIROTORPATHFOLLOWERSETTINGSCC_NORTHPID_KP];
	float North_Kd = multirotorpathfollowerSettings.NorthPID[MULTIROTORPATHFOLLOWERSETTINGSCC_NORTHPID_KD];
	float North_Ki = multirotorpathfollowerSettings.NorthPID[MULTIROTORPATHFOLLOWERSETTINGSCC_NORTHPID_KI];
	float North_ilimit = multirotorpathfollowerSettings.NorthPID[MULTIROTORPATHFOLLOWERSETTINGSCC_NORTHPID_ILIMIT];
	
	if (North_Ki > 0.0f) {
		//Integrate with saturation
		integral->position_error_North =
		bound(integral->position_error_North + position_error_North * dT,
			  -North_ilimit / North_Ki,
			  North_ilimit / North_Ki);
	}
	
	
	float acceleration_desired_North = position_error_North*North_Kp+integral->position_error_North*North_Ki+velocity_error_North*North_Kd;
	
	
	/**
	 * Compute desired East Acceleration
	 */
	float East_Kp = multirotorpathfollowerSettings.EastPID[MULTIROTORPATHFOLLOWERSETTINGSCC_EASTPID_KP];
	float East_Kd = multirotorpathfollowerSettings.EastPID[MULTIROTORPATHFOLLOWERSETTINGSCC_EASTPID_KD];
	float East_Ki = multirotorpathfollowerSettings.EastPID[MULTIROTORPATHFOLLOWERSETTINGSCC_EASTPID_KI];
	float East_ilimit = multirotorpathfollowerSettings.EastPID[MULTIROTORPATHFOLLOWERSETTINGSCC_EASTPID_ILIMIT];
	
	if (East_Ki > 0.0f) {
		//Integrate with saturation
		integral->position_error_East =
		bound(integral->position_error_East + position_error_East * dT,
			  -East_ilimit / East_Ki,
			  East_ilimit / East_Ki);
	}
	
	float acceleration_desired_East = position_error_East*East_Kp+integral->position_error_East*East_Ki+velocity_error_East*East_Kd;
	
	
	/**
	 * Compute desired down acceleration
	 */
	
	float Down_Kp = multirotorpathfollowerSettings.DownPID[MULTIROTORPATHFOLLOWERSETTINGSCC_DOWNPID_KP];
	float Down_Ki = multirotorpathfollowerSettings.DownPID[MULTIROTORPATHFOLLOWERSETTINGSCC_DOWNPID_KI];
	float Down_ilimit = multirotorpathfollowerSettings.DownPID[MULTIROTORPATHFOLLOWERSETTINGSCC_DOWNPID_ILIMIT];
	float Down_Kd= multirotorpathfollowerSettings.DownPID[MULTIROTORPATHFOLLOWERSETTINGSCC_DOWNPID_KD];
	
	
	
	//Integrate with bound. Make integral leaky for better performance.
	//Approximately 30s time constant.
	
	if (Down_Ki > 0.0f) {
		integral->position_error_Down =
		bound(integral->position_error_Down + position_error_Down * dT,
			  -Down_ilimit / Down_Ki,
			  Down_ilimit / Down_Ki) * (1.0f - 1.0f / (1.0f + 30.0f / dT));
	}
	
	float acceleration_desired_Down = position_error_Down*Down_Kp+integral->position_error_Down*Down_Ki+velocity_error_Down*Down_Kd;
	
	
	//==================================
	//Determine throttle, pitch, and roll commands
	//================================
	
	//NEED TO ACCESS CURRENT ATTITUDE
	float desired_Roll = -(1/GRAV)*(acceleration_desired_North*sinf(attitudeActual_Yaw)-acceleration_desired_East*cosf(attitudeActual_Yaw));
	float desired_Pitch = -(1/GRAV)*(acceleration_desired_North*cosf(attitudeActual_Yaw)+acceleration_desired_East*sinf(attitudeActual_Yaw));
	
	
	//Saturate pitch command
	float pitchlimit_min = multirotorpathfollowerSettings.PitchLimit[MULTIROTORPATHFOLLOWERSETTINGSCC_PITCHLIMIT_MIN];
	float pitchlimit_max = multirotorpathfollowerSettings.PitchLimit[MULTIROTORPATHFOLLOWERSETTINGSCC_PITCHLIMIT_MAX];
	
	stabDesired.Pitch = bound(desired_Pitch, pitchlimit_min, pitchlimit_max);
	
	//Saturate roll command
	float rolllimit_min = multirotorpathfollowerSettings.RollLimit[MULTIROTORPATHFOLLOWERSETTINGSCC_ROLLLIMIT_MIN];
	float rolllimit_max = multirotorpathfollowerSettings.RollLimit[MULTIROTORPATHFOLLOWERSETTINGSCC_ROLLLIMIT_MAX];
	
	stabDesired.Roll = bound(desired_Roll, rolllimit_min, rolllimit_max);
	
	float throttlelimit_neutral = multirotorpathfollowerSettings.ThrottleLimit[MULTIROTORPATHFOLLOWERSETTINGSCC_THROTTLELIMIT_NEUTRAL];
	float throttlelimit_min = multirotorpathfollowerSettings.ThrottleLimit[MULTIROTORPATHFOLLOWERSETTINGSCC_THROTTLELIMIT_MIN];
	float throttlelimit_max = multirotorpathfollowerSettings.ThrottleLimit[MULTIROTORPATHFOLLOWERSETTINGSCC_THROTTLELIMIT_MAX];
	
	// set throttle
	stabDesired.Throttle = bound(acceleration_desired_Down + throttlelimit_neutral, throttlelimit_min, throttlelimit_max);

	// Set yaw to 0, as it's not important to holonomic flight
	stabDesired.Yaw =0;
	
	// Set the stabilization mode for each axis
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_ROLL] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_PITCH] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	stabDesired.StabilizationMode[STABILIZATIONDESIRED_STABILIZATIONMODE_YAW] = STABILIZATIONDESIRED_STABILIZATIONMODE_ATTITUDE;
	
	// Update the UAVO
	StabilizationDesiredSet(&stabDesired);

}


/**
 * Bound input value between limits
 */
static float bound(float val, float min, float max)
{
	if (val < min) {
		val = min;
	} else if (val > max) {
		val = max;
	}
	return val;
}

//Triggered by changes in FlightStatus
static void FlightStatusUpdatedCb(UAVObjEvent * ev){
	flightStatusUpdate = true;
}

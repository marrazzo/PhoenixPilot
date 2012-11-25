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
#define GEE 9.81f
// Private types

// Private variables
static bool followerEnabled = false;
static xTaskHandle multirotorPathFollowerTaskHandle;
static MultiRotorPathFollowerSettingsCCData multirotorpathfollowerSettings;

static float northVelIntegral = 0;
static float eastVelIntegral = 0;
static float downVelIntegral = 0;

static float headingIntegral = 0;
static float speedIntegral = 0;
static float accelIntegral = 0;
static float powerIntegral = 0;

// Private functions
static void multirotorPathFollowerTask(void *parameters);
static void updateMultiRotorDesiredAttitude();

/**
 * Initialise the module, called on startup
 * \returns 0 on success or -1 if initialisation failed
 */
int32_t MultiRotorPathFollowerStart()
{
	if (followerEnabled) {
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
	
	MultiRotorPathFollowerSettingsCCGet(&multirotorpathfollowerSettings);
	
	// Main task loop
	lastUpdateTime = xTaskGetTickCount();
	while (1) {
		// Wait until required amount of time has passed
		vTaskDelayUntil(&lastUpdateTime, multirotorpathfollowerSettings.UpdatePeriod / portTICK_RATE_MS);
		
		// Get FlighStatus UAVO
		FlightStatusData flightStatus;
		FlightStatusGet(&flightStatus);
		

		// Check the flightmode
		switch(flightStatus.FlightMode) {
			case FLIGHTSTATUS_FLIGHTMODE_POSITIONHOLD:
			case FLIGHTSTATUS_FLIGHTMODE_RETURNTOHOME:
				updateMultiRotorDesiredAttitude();
				break;
			default:
				// Reset integrals
				// TODO: Be cleaner and get rid of global variables
				northVelIntegral = 0;
				eastVelIntegral = 0;
				downVelIntegral = 0;
				headingIntegral = 0;
				speedIntegral = 0;
				accelIntegral = 0;
				powerIntegral = 0;

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
static void updateMultiRotorDesiredAttitude()
{


}

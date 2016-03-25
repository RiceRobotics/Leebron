/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2014, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"

void driveToLoc(Ricelocation* target);

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the scheduler is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {

	printf("Begin Teleop\n\r");
	visionPid->running = 0;
	visionPid->integral = 0;
	while (1) {
		getJoystickForDriveTrain();

		if(joystickGetDigital(1, 8, JOY_RIGHT)) fsmOverride = 1;
		else if(joystickGetDigital(1, 8, JOY_LEFT)) fsmOverride = 0;

		if(fsmOverride) {
			if(joystickGetDigital(1, 6, JOY_UP)) MOTIntake->out = 127;
			else if(joystickGetDigital(1, 6, JOY_DOWN)) MOTIntake->out = -127;
			else MOTIntake->out = 0;

			if(joystickGetDigital(1, 7, JOY_UP)) MOTConveyor->out = 127;
			else if(joystickGetDigital(1, 7, JOY_DOWN)) MOTConveyor->out = -127;
			else MOTConveyor->out = 0;

			if(joystickGetDigital(1, 7, JOY_RIGHT)) MOTMagazine->out = 127;
			else if(joystickGetDigital(1, 7, JOY_LEFT)) MOTMagazine->out = -127;
			else MOTMagazine->out = 0;

			if(joystickGetDigital(1, 5, JOY_UP)) {
				MOTHammer->out = 127;
				MOTMjolnir->out = 127;
			}
			else {
				MOTHammer->out = 0;
				MOTMjolnir->out = 0;
			}
		}
		else {
			if(joystickGetDigital(1, 6, JOY_UP)) intakeDir = 1;
			else if(joystickGetDigital(1, 6, JOY_DOWN)) intakeDir = -1;
			else intakeDir = 0;

			if(joystickGetDigital(1, 5, JOY_UP)) launch = 1;
			else launch = 0;

			//			fullFSM(intakeDir, launch);
		}

		if(joystickGetDigital(2, 7, JOY_UP)) vision();
//		if(joystickGetDigital(2, 7, JOY_LEFT)) saveLocation(targetLoc, currentLoc);
		if(joystickGetDigital(2, 7, JOY_RIGHT)) driveToLoc(targetLoc);
		if(joystickGetDigital(2, 7, JOY_DOWN)) autonomous();

		if(joystickGetDigital(2, 8, JOY_UP)) pidDrive(24, 5000);
		if(joystickGetDigital(2, 8, JOY_LEFT)) pidTurn(gyro->value + 90, 5000);
		if(joystickGetDigital(2, 8, JOY_RIGHT)) pidTurn(gyro->value - 90, 5000);
		//		if(joystickGetDigital(2, 8, JOY_UP)) autonomous();
		if(joystickGetDigital(2, 8, JOY_DOWN)) pidDrive(-24, 5000);

		printf("override: %d\n\r", fsmOverride);

		//		printf("Angle: %4d, (x, y): (%4f, %4f), (%4f, %4f), (%4f, %4f)\n\r",
		//				rps->currentLoc->angle, rps->currentLoc->x, rps->currentLoc->y,
		//				rps->currentLoc->xLeftRaw, rps->currentLoc->yLeftRaw,
		//				rps->currentLoc->xRightRaw, rps->currentLoc->yRightRaw);
		delay(20);
	}
}

void fullFSM(void* ignore) {
	static int intakeStateLast = 0, hammerStateLast = 0;
	static long launchTime = 0;

	while(1) {
		if(fsmOverride) {
			if(joystickGetDigital(1, 6, JOY_UP)) MOTIntake->out = 127;
			else if(joystickGetDigital(1, 6, JOY_DOWN)) MOTIntake->out = -127;
			else MOTIntake->out = 0;

			if(joystickGetDigital(1, 7, JOY_UP)) MOTConveyor->out = 127;
			else if(joystickGetDigital(1, 7, JOY_DOWN)) MOTConveyor->out = -127;
			else MOTConveyor->out = 0;

			if(joystickGetDigital(1, 7, JOY_RIGHT)) MOTMagazine->out = 127;
			else if(joystickGetDigital(1, 7, JOY_LEFT)) MOTMagazine->out = -127;
			else MOTMagazine->out = 0;

			if(joystickGetDigital(1, 5, JOY_UP)) {
				MOTHammer->out = 127;
				MOTMjolnir->out = 127;
			}
			else {
				MOTHammer->out = 0;
				MOTMjolnir->out = 0;
			}
		}
		else {
			intakeState = ANAIntake->value < ANAIntakeThreshold;
			conveyorState = ANAConveyor->value < ANAConveyorThreshold;
			hammerState = ANAHammer->value < ANAHammerThreshold;

			if(intakeDir == 1) {
				MOTIntake->out = 127;
				if(!conveyorState && intakeState) {
					MOTConveyor->out = 127;
				}
				else {
					MOTConveyor->out = 0;
				}
				//		if(intakeState && !intakeStateLast) numBalls++;
			}
			else if(intakeDir == -1) {
				MOTIntake->out = -127;
				MOTConveyor->out = -127;
				//		if(!intakeState && intakeStateLast) numBalls--;
			}
			else {
				MOTIntake->out = 0;
				MOTConveyor->out = 0;
			}

			if(launch) {
				if(hammerState) {
					MOTMjolnir->out = 90;
					MOTHammer->out = 90;
					MOTMagazine->out = 0;
				}
				else if(millis() - launchTime < 1000) {
					MOTMjolnir->out = 90;
					MOTHammer->out = 90;
					MOTMagazine->out = 80;
					MOTConveyor->out = 80;
				}
				else {
					MOTMjolnir->out = 0;
					MOTHammer->out = 0;
					MOTMagazine->out = 127;
					if(!conveyorState) MOTConveyor->out = 127;
					else MOTConveyor->out = 0;
					if(intakeState) MOTIntake->out = 127;
					else MOTIntake->out = 0;
					if(!hammerState && hammerStateLast) {
						launchTime = millis();
						numBalls--;
					}
				}
			}
			else {
				MOTMjolnir->out = 0;
				MOTHammer->out = 0;
				MOTMagazine->out = 0;
			}

			//	if(numBalls < 0 || joystickGetDigital(1, 8, JOY_LEFT)) numBalls = 0;

			//	printf("IntakeState|Last: %d|%d, ANAIntake: %4d, ANAConveyor: %4d, ANAHammer: %4d, numBalls: %d\r",
			//			intakeState, intakeStateLast, ANAIntake->value, ANAConveyor->value, ANAHammer->value, numBalls);

			intakeStateLast = intakeState;
			hammerStateLast = hammerState;
		}
	}
}

void driveToLoc(Ricelocation* target) {
	Ricelocation* relativeTarget = initRicelocation(
			target->x - rps->currentLoc->x, target->y - rps->currentLoc->y, target->angle);

	float angleToTarget = atan2f(relativeTarget->y, relativeTarget->x) * 180.0 / MATH_PI;
	float distanceToTarget = sqrt(pow(relativeTarget->x, 2) + pow(relativeTarget->y, 2));

	pidTurn(angleToTarget, 5000);
	pidDrive(distanceToTarget, 8000);
	pidTurn(relativeTarget->angle, 5000);
}

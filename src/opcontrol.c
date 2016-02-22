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

	int intakeDir, launch;

	resetRicencoder();
	while (1) {
		getJoystickForDriveTrain();

		if(joystickGetDigital(1, 6, JOY_UP)) intakeDir = 1;
		else if(joystickGetDigital(1, 6, JOY_DOWN)) intakeDir = -1;
		else intakeDir = 0;

		if(joystickGetDigital(1, 5, JOY_UP)) launch = 1;
		else launch = 0;

		if(joystickGetDigital(1, 8, JOY_UP) && !driveLeftPid->running && !driveRightPid->running) {
			driveLeftPid->running = 1;
			driveRightPid->running = 1;
			float inches = 12;
			driveLeftPid->setPoint = ENCDTLeft->adjustedValue + inches / (4.0*MATH_PI) * ENCDTLeft->ticksPerRev;
			driveRightPid->setPoint = ENCDTRight->adjustedValue + inches / (4.0*MATH_PI) * ENCDTRight->ticksPerRev;
		}
		if(joystickGetDigital(1, 8, JOY_DOWN) && !driveLeftPid->running && !driveRightPid->running) {
			driveLeftPid->running = 1;
			driveRightPid->running = 1;
			float inches = -12;
			driveLeftPid->setPoint = ENCDTLeft->adjustedValue + inches / (4.0*MATH_PI) * ENCDTLeft->ticksPerRev;
			driveRightPid->setPoint = ENCDTRight->adjustedValue + inches / (4.0*MATH_PI) * ENCDTRight->ticksPerRev;
		}
		if(driveLeftPid->running && driveRightPid->running) {
			if(joystickGetDigital(1, 7, JOY_UP) || driveLeftPid->atSetpoint || driveRightPid->atSetpoint) {
				driveLeftPid->running = 0;
				driveRightPid->running = 0;
				driveLeftPid->atSetpoint = 0;
				driveRightPid->atSetpoint = 0;
				driveLeftPid->integral = 0;
				driveRightPid->integral = 0;
				MOTDTFrontLeft->out = 0;
				MOTDTMidLeft->out = 0;
				MOTDTFrontRight->out = 0;
				MOTDTMidRight->out = 0;
			}
			else {
				MOTDTFrontLeft->out = driveLeftPid->output;
				MOTDTMidLeft->out = driveLeftPid->output;
				MOTDTFrontRight->out = driveRightPid->output;
				MOTDTMidRight->out = driveRightPid->output;
			}
		}

		if(joystickGetDigital(1, 8, JOY_LEFT) && !gyroPid->running) {
			gyroPid->running = 1;
			gyroPid->setPoint = gyro->value + 90;
		}
		if(joystickGetDigital(1, 8, JOY_RIGHT) && !gyroPid->running) {
			gyroPid->running = 1;
			gyroPid->setPoint = gyro->value - 90;
		}
		if(gyroPid->running) {
			if(joystickGetDigital(1, 7, JOY_DOWN) || gyroPid->atSetpoint) {
				gyroPid->running = 0;
				gyroPid->atSetpoint = 0;
				gyroPid->integral = 0;
				MOTDTFrontLeft->out = 0;
				MOTDTMidLeft->out = 0;
				MOTDTFrontRight->out = 0;
				MOTDTMidRight->out = 0;
			}
			else {
				MOTDTFrontLeft->out = -gyroPid->output;
				MOTDTMidLeft->out = -gyroPid->output;
				MOTDTFrontRight->out = gyroPid->output;
				MOTDTMidRight->out = gyroPid->output;
			}
		}

		//		printf("Gyro: %4d/%4d, pidRunning: %1d, error: %3.1f, atSetpoint: %1d, "
		//				"output: %4d = %4.1f + %4.1f + %4.1f\n\r",
		//				gyro->value, gyroPid->setPoint, gyroPid->running, gyroPid->error, gyroPid->atSetpoint,
		//				gyroPid->output, gyroPid->kP*gyroPid->error, gyroPid->kI*gyroPid->integral, gyroPid->kD*gyroPid->derivative);

		printf("Enc: %4d/%4d, pidRunning: %1d, error: %3.1f, atSetpoint: %1d, "
				"output: %4d = %4.1f + %4.1f + %4.1f\n\r",
				ENCDTLeft->adjustedValue, driveLeftPid->setPoint, driveLeftPid->running, driveLeftPid->error, driveLeftPid->atSetpoint,
				driveLeftPid->output, driveLeftPid->kP*driveLeftPid->error, driveLeftPid->kI*driveLeftPid->integral, driveLeftPid->kD*driveLeftPid->derivative);
		fullFSM(intakeDir, launch);
		delay(20);
	}
}

void fullFSM(int intakeDir, int launch) {
	int intakeState, conveyorState, hammerState;
	static int intakeStateLast = 0, hammerStateLast = 0;
	static int numBalls = 0;

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
		if(intakeState && !intakeStateLast) numBalls++;
	}
	else if(intakeDir == -1) {
		MOTIntake->out = -127;
		MOTConveyor->out = -127;
		if(!intakeState && intakeStateLast) numBalls--;
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
		else {
			MOTMjolnir->out = 0;
			MOTHammer->out = 0;
			MOTMagazine->out = 127;
			if(!conveyorState) MOTConveyor->out = 127;
			else MOTConveyor->out = 0;
			if(intakeState) MOTIntake->out = 127;
			else MOTIntake->out = 0;
			if(!hammerState && hammerStateLast) numBalls--;
		}
	}
	else {
		MOTMjolnir->out = 0;
		MOTHammer->out = 0;
		MOTMagazine->out = 0;
	}

	if(numBalls < 0 || joystickGetDigital(1, 8, JOY_LEFT)) numBalls = 0;

	//	if(intakeState != intakeStateLast) {
	//		printf("\nIntakeState|Last: %d|%d, ANAIntake: %4d, ANAConveyor: %4d, ANAHammer: %4d, numBalls: %d\n\r",
	//				intakeState, intakeStateLast, ANAIntake->value, ANAConveyor->value, ANAHammer->value, numBalls);
	//	}
	//	else
	//		printf("IntakeState|Last: %d|%d, ANAIntake: %4d, ANAConveyor: %4d, ANAHammer: %4d, numBalls: %d\r",
	//						intakeState, intakeStateLast, ANAIntake->value, ANAConveyor->value, ANAHammer->value, numBalls);
	intakeStateLast = intakeState;
	hammerStateLast = hammerState;
}



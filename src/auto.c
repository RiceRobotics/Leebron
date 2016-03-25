/** @file auto.c
 * @brief File for autonomous code
 *
 * This file should contain the user autonomous() function and any functions related to it.
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
 * Runs the user autonomous code. This function will be started in its own task with the default
 * priority and stack size whenever the robot is enabled via the Field Management System or the
 * VEX Competition Switch in the autonomous mode. If the robot is disabled or communications is
 * lost, the autonomous task will be stopped by the kernel. Re-enabling the robot will restart
 * the task, not re-start it from where it left off.
 *
 * Code running in the autonomous task cannot access information from the VEX Joystick. However,
 * the autonomous function can be invoked from another task if a VEX Competition Switch is not
 * available, and it can access joystick information if called in this way.
 *
 * The autonomous task may exit, unlike operatorControl() which should never exit. If it does
 * so, the robot will await a switch to another mode or disable/enable cycle.
 */
void autonomous() {
	intakeDir = 0;
	launch = 0;
	pidDrive(-28, 5000);
	delay(500);
	//	pidTurn(gyro->value - 20, 5000);
	autonomousTask(AUTOTURNTIME, -1, 30, 500);
	delay(500);
	int startTime = millis();
	while(numBalls > -4 && millis() - startTime < 15000) {
		launch = 1;
		delay(20);
	}
	launch = 0;
	delay(500);
	pidDrive(-6, 2500);
	delay(500);
	pidTurn(gyro->value + 60, 5000);
	delay(500);
	intakeDir = 1;
	autonomousTask(AUTODRIVETIME, NULL, 60, 1000);
	delay(500);
	autonomousTask(AUTODRIVETIME, NULL, -60, 750);
	delay(500);
	autonomousTask(AUTODRIVETIME, NULL, 60, 1000);
	delay(500);
	autonomousTask(AUTODRIVETIME, NULL, -60, 750);
	delay(500);
	autonomousTask(AUTODRIVETIME, NULL, 60, 1000);
	delay(500);
	autonomousTask(AUTODRIVETIME, NULL, -60, 750);
	delay(500);
	intakeDir = 0;
	pidTurn(gyro->value - 80, 5000);
//	autonomousTask(AUTOTURNTIME, -1, 30, 300);
	delay(1000);
//	pidDrive(6, 2500);
	delay(500);
	startTime = millis();
	while(millis() - startTime < 15000) {
		launch = 1;
		delay(20);
	}
}

void pidDrive(float distance, long timeout) {
	long startTime = millis();
	driveLeftPid->running = 1;
	driveRightPid->running = 1;
	driveLeftPid->setPoint = ENCDTLeft->adjustedValue + distance / (4.0*MATH_PI) * ENCDTLeft->ticksPerRev;
	driveRightPid->setPoint = ENCDTRight->adjustedValue + distance / (4.0*MATH_PI) * ENCDTRight->ticksPerRev;
	printf("\n");
	while(driveLeftPid->running && driveRightPid->running && millis() - startTime < timeout) {
		if(joystickGetDigital(2, 7, JOY_DOWN) || (driveLeftPid->atSetpoint && driveRightPid->atSetpoint)) {
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
		printf("Enc: %4d/%4d, pidRunning: %1d, error: %3.1f, atSetpoint: %1d, "
				"output: %4d = %4.1f + %4.1f + %4.1f\r",
				ENCDTLeft->adjustedValue, driveLeftPid->setPoint, driveLeftPid->running, driveLeftPid->error, driveLeftPid->atSetpoint,
				driveLeftPid->output, driveLeftPid->kP*driveLeftPid->error, driveLeftPid->kI*driveLeftPid->integral, driveLeftPid->kD*driveLeftPid->derivative);
	}
}

//Left is positive, right is negative
void pidTurn(int angle, long timeout) {
	long startTime = millis();
	gyroPid->running = 1;
	gyroPid->setPoint = angle;
	printf("\n");
	while(gyroPid->running && millis() - startTime < timeout) {
		if(joystickGetDigital(2, 7, JOY_DOWN) || gyroPid->atSetpoint) {
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
//		printf("Gyro: %4d/%4d, pidRunning: %1d, error: %3.1f, atSetpoint: %1d, "
//				"output: %4d = %4.1f + %4.1f + %4.1f\r",
//				gyro->value, gyroPid->setPoint, gyroPid->running, gyroPid->error, gyroPid->atSetpoint,
//				gyroPid->output, gyroPid->kP*gyroPid->error, gyroPid->kI*gyroPid->integral, gyroPid->kD*gyroPid->derivative);
	}
	gyroPid->running = 0;
	gyroPid->integral = 0;
}

void vision() {
	visionPid->running = 1;
	int targetAngle = gyro->value;		//UART angle goeth here.
	seesBall = 0;									//UART ball in sight
	int turnPow = 0;
	int drivePow = 30;
	for(ever) {
		if(joystickGetDigital(2, 7, JOY_DOWN)) operatorControl();
		char* sInput = fgets(uartIn, 6, uart1);
		if(sInput) {
			//			printf("%6s\n\r", sInput);
			char* sAngle = strtok(sInput, " ");
			char* sBalls = strtok(NULL, " ");
			seesBall = atoi(sBalls);
			if(seesBall)
				targetAngle = gyro->value - atoi(sAngle);		//- because raspi angle + is on right, and gyro angle + is on left
			printf("Angle: %3d, %d\n\r", atoi(sAngle), seesBall);
		}
		visionPid->setPoint = targetAngle;
		turnPow = seesBall ? visionPid->output : 0;
		//		printf("Targets: %d/%d degrees, %d, pow: %d\n\r", gyro->value, targetAngle, seesBall, turnPow);
		//		printf("Gyro: %d/%d, atSetpoint: %d, output: %d\n\r", gyro->value, gyroPid->setPoint, gyroPid->atSetpoint, gyroPid->output);
		if(seesBall) {
			if(!visionPid->atSetpoint) {
				MOTDTFrontLeft->out = -turnPow;
				MOTDTFrontRight->out = turnPow;
				MOTDTMidLeft->out = -turnPow;
				MOTDTMidRight->out = turnPow;
			}
			else {
				//				fullFSM(1, 0);
				//				MOTDTFrontLeft->out = drivePow;
				//				MOTDTFrontRight->out = drivePow;
				//				MOTDTMidLeft->out = drivePow;
				//				MOTDTMidRight->out = drivePow;
				MOTDTFrontLeft->out = 0;
				MOTDTFrontRight->out = 0;
				MOTDTMidLeft->out = 0;
				MOTDTMidRight->out = 0;
			}
		}
		else {
			targetAngle = gyro->value;
			if(intakeState && conveyorState) {
				operatorControl();
			}
			else {
				turnPow = 30;
				//				MOTDTFrontLeft->out = -turnPow;
				//				MOTDTFrontRight->out = turnPow;
				//				MOTDTMidLeft->out = -turnPow;
				//				MOTDTMidRight->out = turnPow;
				MOTDTFrontLeft->out = 0;
				MOTDTFrontRight->out = 0;
				MOTDTMidLeft->out = 0;
				MOTDTMidRight->out = 0;
			}
		}
		delay(20);
	}
	MOTDTFrontLeft->out = 0;
	MOTDTFrontRight->out = 0;
	MOTDTMidLeft->out = 0;
	MOTDTMidRight->out = 0;
}

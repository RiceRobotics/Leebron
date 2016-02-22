/** @file init.c
 * @brief File for initialization code
 *
 * This file should contain the user initialize() function and any functions related to it.
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

Gyro test;

/*
 * Runs pre-initialization code. This function will be started in kernel mode one time while the
 * VEX Cortex is starting up. As the scheduler is still paused, most API functions will fail.
 *
 * The purpose of this function is solely to set the default pin modes (pinMode()) and port
 * states (digitalWrite()) of limit switches, push buttons, and solenoids. It can also safely
 * configure a UART port (usartOpen()) but cannot set up an LCD (lcdInit()).
 */
void initializeIO() {
	usartInit(uart1, 115200, SERIAL_8N1);
}

/*
 * Runs user initialization code. This function will be started in its own task with the default
 * priority and stack size once when the robot is starting up. It is possible that the VEXnet
 * communication link may not be fully established at this time, so reading from the VEX
 * Joystick may fail.
 *
 * This function should initialize most sensors (gyro, encoders, ultrasonics), LCDs, global
 * variables, and IMEs.
 *
 * This function must exit relatively promptly, or the operatorControl() and autonomous() tasks
 * will not start. An autonomous mode selection menu like the pre_auton() in other environments
 * can be implemented in this task if desired.
 */
void initialize() {
	uartIn = malloc(sizeof(char)*16);
	driveTrainStyle = DTFOURWHEELS;
	controlStyle = CTCHEEZYDRIVE;
	riceBotInitialize();

	MOTDTFrontLeft = initRicemotor(2, -1);
	MOTDTMidLeft = initRicemotor(8, 1);					//Y-split

	MOTDTFrontRight = initRicemotor(3, 1);
	MOTDTMidRight = initRicemotor(9, -1);				//Y-split

	MOTIntake = initRicemotor(6, 1);
	MOTConveyor = initRicemotor(7, -1);
	MOTMagazine = initRicemotor(5, -1);
	MOTMjolnir = initRicemotor(4, -1);
	MOTHammer = initRicemotor(10, -1);

	ANAIntake = initRicesensorAnalog(2, true);
	ANAConveyor = initRicesensorAnalog(3, true);
	ANAHammer = initRicesensorAnalog(4, true);

	ANAIntakeThreshold = -30;
	ANAConveyorThreshold = -150;
	ANAHammerThreshold = -100;

	gyro = initRicegyro(1, 198);

	imeInitializeAll();
	ENCDTLeft = initRicencoderIME(627.2, 1, 0, false);
	ENCDTRight = initRicencoderIME(627.2, 1, 1, true);

	Ricemotor* array[2] = {MOTDefault, MOTDefault};
	driveLeftPid = initRicepid(&ENCDTLeft->adjustedValue, 35, .09, .0005, .05, array);
	driveRightPid = initRicepid(&ENCDTRight->adjustedValue, 35, .09, .0005, .05, array);
	gyroPid = initRicepid(&gyro->value, 1, .25, .003, 0, array);

	taskCreate(IOTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_HIGHEST);
	taskCreate(PidTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
}

void vision() {
//	gyroReset(gyro->g);
	gyroPid->running = 1;
	int targetAngle = gyro->value;		//UART angle goeth here.
	int seesBall = 0;							//UART ball in sight
	int turnPow = 20;
	int drivePow = 30;
	for(ever) {
		//		printf("Targets: %d/%d degrees, %d\n\r", gyro->value, targetAngle, seesBall);
		if(joystickGetDigital(1, 8, JOY_UP)) operatorControl();
		char* sInput = fgets(uartIn, 6, uart1);
		if(sInput) {
			char* sAngle = strtok(sInput, " ");
			char* sBalls = strtok(NULL, " ");
			seesBall = atoi(sBalls);
			if(seesBall)
				targetAngle = gyro->value + atoi(sAngle);
		}
		gyroPid->setPoint = targetAngle;
		turnPow = seesBall ? gyroPid->output : 0;
		printf("Gyro: %d/%d, atSetpoint: %d, output: %d\n\r", gyro->value, gyroPid->setPoint, gyroPid->atSetpoint, gyroPid->output);
		if(!gyroPid->atSetpoint) {
			MOTDTFrontLeft->out = -turnPow;
			MOTDTFrontRight->out = turnPow;
			MOTDTBackLeft->out = -turnPow;
			MOTDTBackRight->out = turnPow;
		}
		else if(seesBall){
			MOTDTFrontLeft->out = drivePow;
			MOTDTFrontRight->out = drivePow;
			MOTDTBackLeft->out = drivePow;
			MOTDTBackRight->out = drivePow;
		}
		if(!seesBall) {
			targetAngle = gyro->value;
		}
		delay(20);
	}
	MOTDTFrontLeft->out = 0;
	MOTDTFrontRight->out = 0;
	MOTDTBackLeft->out = 0;
	MOTDTBackRight->out = 0;
}

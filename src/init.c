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
	intakeDir = 0;
	launch = 0;
	numBalls = 0;

	intakeState = 0;
	conveyorState = 0;
	hammerState = 0;

	seesBall = 0;

	fsmOverride = 0;

	uartIn = malloc(sizeof(char)*16);
	driveTrainStyle = DTFOURWHEELS;
	controlStyle = CTCHEEZYDRIVE;
	rpsActive = 1;
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

	ANAIntake = initRicesensorAnalog(2, false);
	ANAConveyor = initRicesensorAnalog(3, false);
	ANAHammer = initRicesensorAnalog(4, false);

//	ANAIntakeThreshold = -30;
//	ANAConveyorThreshold = -150;
//	ANAHammerThreshold = -200;
	ANAIntakeThreshold = 2990;
	ANAConveyorThreshold = 2650;
	ANAHammerThreshold = 2900;

	gyro = initRicegyro(1, 196);

	imeInitializeAll();
	ENCDTLeft = initRicencoderIME(627.2, 1, 0, false);
	ENCDTRight = initRicencoderIME(627.2, 1, 1, true);

	Ricemotor* array[2] = {MOTDefault, MOTDefault};
	driveLeftPid = initRicepid(&ENCDTLeft->adjustedValue, 40, .05, .0002, 0, array);
	driveRightPid = initRicepid(&ENCDTRight->adjustedValue, 40, .05, .0002, 0, array);

	gyroPid = initRicepid(&gyro->value, 2, .25, .003, 0, array);

	visionPid = initRicepid(&gyro->value, 2, 1.5, .03, .03, array);

	taskCreate(IOTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_HIGHEST);
	taskCreate(fullFSM, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
	taskCreate(PidTask, TASK_DEFAULT_STACK_SIZE, NULL, TASK_PRIORITY_DEFAULT);
	printf("Init complete\n\r");
}

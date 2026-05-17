/*
 * SimpleParser.h
 *
 *  Created on: Aug 28, 2024
 *      Author: Szymon
 */

#ifndef INC_SIMPLEPARSER_H_
#define INC_SIMPLEPARSER_H_

#include "main.h"
#include "string.h"
#include "RingBuffer.h"
#include "Line_Follower.h"

#define ENDLINE '\n'
#define MY_NAME "GRUZIK3.0"
#define PARSER_LINE_BUFFER_SIZE 96u

typedef enum
{
	TELEMETRY_OFF = 0,
	TELEMETRY_ODOM = 1,
	TELEMETRY_DEBUG = 2
} TelemetryMode_t;

extern volatile uint8_t TelemetryMode;
extern volatile uint8_t TireCleaningActive;
extern volatile uint8_t ManualDriveActive;

void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *ReceivedData);
void Parser_Parse(uint8_t *ReceivedData, LineFollower_t *LineFollower);
void App_Controll(char RxData, LineFollower_t *LineFollower);
void Parser_FinishMappingAutoClosed(LineFollower_t *LineFollower);
void Parser_ServiceManualDriveTimeout(LineFollower_t *LineFollower);
uint8_t Parser_ServicePendingBluetoothName(void);

#endif /* INC_SIMPLEPARSER_H_ */

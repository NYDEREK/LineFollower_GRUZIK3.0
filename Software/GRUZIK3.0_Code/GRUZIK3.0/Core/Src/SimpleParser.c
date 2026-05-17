/*
 * SimpleParser.c
 *
 * UART command parser for the renewed GRUZIK3.0 firmware.
 */

#include "SimpleParser.h"
#include "Line_Follower.h"
#include "app_fatfs.h"
#include "gpio.h"
#include "map.h"
#include "robot_config.h"
#include "robot_settings.h"
#include "tim.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char My_Name[32] = "GRUZIK3.0";

extern FRESULT FatFsResult;
extern FATFS SdFatFs;
extern FIL SdCardFile;
extern uint8_t SDReadingReady;
extern uint8_t RxData;
extern Map_t map;

volatile uint8_t TelemetryMode = TELEMETRY_OFF;
volatile uint8_t TireCleaningActive = 0u;
volatile uint8_t ManualDriveActive = 0u;

static uint8_t SdFileOpen = 0u;
static uint16_t MapUploadPointCount = 0u;
static uint16_t MapUploadExpectedCount = 0u;
static float TireCleanSpeed = 170.0f;
static uint32_t ManualDriveLastTick = 0u;

static void SendFatFsError(const char *prefix, const char *operation, const char *file_name, FRESULT result);
static uint8_t SdEnsureMounted(const char *context);

static void UartSend(const char *text)
{
	HAL_UART_Transmit(&hlpuart1, (uint8_t *)text, strlen(text), 500);
}

static const char *SettingsStatusText(RobotSettingsStatus_t status)
{
	switch(status)
	{
	case ROBOT_SETTINGS_OK:
		return "ok";
	case ROBOT_SETTINGS_EMPTY:
		return "empty";
	case ROBOT_SETTINGS_INVALID:
		return "invalid";
	default:
		return "flash_error";
	}
}

static char *NextValue(void)
{
	return strtok(NULL, ",");
}

static uint8_t ParseFloatValue(float *out)
{
	char *parse_pointer = NextValue();
	if((parse_pointer == NULL) || (strlen(parse_pointer) == 0u) || (strlen(parse_pointer) >= 32u))
	{
		return 0u;
	}

	*out = strtof(parse_pointer, NULL);
	return 1u;
}

static void TrimText(char *text)
{
	size_t len = strlen(text);
	while((len > 0u) && ((text[len - 1u] == ' ') || (text[len - 1u] == '\t') ||
						 (text[len - 1u] == '\r') || (text[len - 1u] == '\n')))
	{
		text[len - 1u] = '\0';
		len--;
	}

	char *start = text;
	while((*start == ' ') || (*start == '\t'))
	{
		start++;
	}

	if(start != text)
	{
		memmove(text, start, strlen(start) + 1u);
	}
}

static uint8_t BluetoothNameIsValid(const char *name)
{
	size_t len = strlen(name);
	if((len == 0u) || (len > ROBOT_BT_NAME_MAX_LEN))
	{
		return 0u;
	}

	for(size_t i = 0u; i < len; i++)
	{
		char c = name[i];
		uint8_t allowed = (((c >= 'A') && (c <= 'Z')) ||
						   ((c >= 'a') && (c <= 'z')) ||
						   ((c >= '0') && (c <= '9')) ||
						   (c == ' ') || (c == '_') || (c == '-') || (c == '.'));
		if(allowed == 0u)
		{
			return 0u;
		}
	}

	return 1u;
}

static uint8_t ReadBluetoothNameArgument(char *name, size_t name_size)
{
	char *value = strtok(NULL, "\r\n");
	if(value == NULL)
	{
		UartSend("BT_NAME_ERROR,empty\r\n");
		return 0u;
	}

	snprintf(name, name_size, "%s", value);
	TrimText(name);
	if(BluetoothNameIsValid(name) == 0u)
	{
		UartSend("BT_NAME_ERROR,bad_name\r\n");
		return 0u;
	}

	return 1u;
}

static uint8_t BluetoothPendingNameWrite(const char *name)
{
	if(SdEnsureMounted(ROBOT_BT_NAME_FILE) == 0u)
	{
		return 0u;
	}

	FIL file;
	FatFsResult = f_open(&file, ROBOT_BT_NAME_FILE, FA_WRITE | FA_CREATE_ALWAYS);
	if(FatFsResult != FR_OK)
	{
		SendFatFsError("BT_NAME_ERROR", "open_write", ROBOT_BT_NAME_FILE, FatFsResult);
		return 0u;
	}

	UINT written = 0u;
	FatFsResult = f_write(&file, name, (UINT)strlen(name), &written);
	if(FatFsResult == FR_OK)
	{
		UINT newline_written = 0u;
		FatFsResult = f_write(&file, "\n", 1u, &newline_written);
	}
	f_close(&file);

	if((FatFsResult != FR_OK) || (written != strlen(name)))
	{
		SendFatFsError("BT_NAME_ERROR", "write", ROBOT_BT_NAME_FILE, FatFsResult);
		return 0u;
	}

	return 1u;
}

static uint8_t BluetoothPendingNameRead(char *name, size_t name_size)
{
	if(SDReadingReady == 0u)
	{
		return 0u;
	}

	if(SdEnsureMounted(ROBOT_BT_NAME_FILE) == 0u)
	{
		return 0u;
	}

	FIL file;
	FatFsResult = f_open(&file, ROBOT_BT_NAME_FILE, FA_READ);
	if(FatFsResult == FR_NO_FILE)
	{
		return 0u;
	}
	if(FatFsResult != FR_OK)
	{
		SendFatFsError("BT_NAME_ERROR", "open_read", ROBOT_BT_NAME_FILE, FatFsResult);
		return 0u;
	}

	UINT bytes_read = 0u;
	char buffer[ROBOT_BT_NAME_MAX_LEN + 8u];
	FatFsResult = f_read(&file, buffer, sizeof(buffer) - 1u, &bytes_read);
	f_close(&file);
	if(FatFsResult != FR_OK)
	{
		SendFatFsError("BT_NAME_ERROR", "read", ROBOT_BT_NAME_FILE, FatFsResult);
		return 0u;
	}

	buffer[bytes_read] = '\0';
	TrimText(buffer);
	if(BluetoothNameIsValid(buffer) == 0u)
	{
		UartSend("BT_NAME_ERROR,pending_bad_name\r\n");
		return 0u;
	}

	snprintf(name, name_size, "%s", buffer);
	return 1u;
}

static uint8_t BluetoothSetBaud(uint32_t baud)
{
	if(hlpuart1.Init.BaudRate == baud)
	{
		return 1u;
	}

	(void)HAL_UART_DeInit(&hlpuart1);
	hlpuart1.Init.BaudRate = baud;
	return (HAL_UART_Init(&hlpuart1) == HAL_OK) ? 1u : 0u;
}

static uint8_t BluetoothResponseOk(const char *response)
{
	return ((strstr(response, "OK") != NULL) ||
			(strstr(response, "Ok") != NULL) ||
			(strstr(response, "ok") != NULL)) ? 1u : 0u;
}

static uint8_t BluetoothSendAtCommand(const char *command, char *response, size_t response_size)
{
	memset(response, 0, response_size);
	(void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)command, (uint16_t)strlen(command), 300);

	uint32_t start_ms = HAL_GetTick();
	size_t used = 0u;
	while(((HAL_GetTick() - start_ms) < ROBOT_BT_AT_TIMEOUT_MS) && (used < (response_size - 1u)))
	{
		uint8_t byte = 0u;
		if(HAL_UART_Receive(&hlpuart1, &byte, 1u, 10u) == HAL_OK)
		{
			response[used++] = (char)byte;
			response[used] = '\0';
			if((BluetoothResponseOk(response) != 0u) || (strstr(response, "ERROR") != NULL))
			{
				break;
			}
		}
	}

	return BluetoothResponseOk(response);
}

static uint8_t BluetoothTryRenameAtBaud(uint32_t baud, const char *name)
{
	char response[96];
	char command[48];

	if(BluetoothSetBaud(baud) == 0u)
	{
		return 0u;
	}
	HAL_Delay(80u);

	if((BluetoothSendAtCommand("AT\r\n", response, sizeof(response)) == 0u) &&
	   (BluetoothSendAtCommand("AT", response, sizeof(response)) == 0u))
	{
		return 0u;
	}

	snprintf(command, sizeof(command), "AT+NAME=%s\r\n", name);
	if(BluetoothSendAtCommand(command, response, sizeof(response)) != 0u)
	{
		return 1u;
	}

	snprintf(command, sizeof(command), "AT+NAME%s\r\n", name);
	if(BluetoothSendAtCommand(command, response, sizeof(response)) != 0u)
	{
		return 1u;
	}

	snprintf(command, sizeof(command), "AT+NAME%s", name);
	return BluetoothSendAtCommand(command, response, sizeof(response));
}

static uint8_t BluetoothApplyName(const char *name, uint32_t *used_baud)
{
	uint8_t ok = 0u;

	(void)HAL_UART_AbortReceive_IT(&hlpuart1);
	HAL_Delay(20u);

	ok = BluetoothTryRenameAtBaud(ROBOT_BT_AT_BAUD_PRIMARY, name);
	if(ok == 0u)
	{
		ok = BluetoothTryRenameAtBaud(ROBOT_BT_AT_BAUD_SECONDARY, name);
		if(ok != 0u)
		{
			*used_baud = ROBOT_BT_AT_BAUD_SECONDARY;
		}
	}
	else
	{
		*used_baud = ROBOT_BT_AT_BAUD_PRIMARY;
	}

	(void)BluetoothSetBaud(ROBOT_BT_DATA_BAUD);
	HAL_Delay(20u);
	(void)HAL_UART_Receive_IT(&hlpuart1, &RxData, 1u);

	return ok;
}

static float ClampFloat(float value, float min_value, float max_value)
{
	if(value > max_value)
	{
		return max_value;
	}
	if(value < min_value)
	{
		return min_value;
	}
	return value;
}

static void SendFatFsError(const char *prefix, const char *operation, const char *file_name, FRESULT result)
{
	char tx[96];
	snprintf(tx, sizeof(tx), "%s,%s,%s,%u\r\n", prefix, operation, file_name, (unsigned int)result);
	UartSend(tx);
}

static uint8_t SdEnsureMounted(const char *context)
{
	FatFsResult = f_mount(&SdFatFs, "", 1);
	if(FatFsResult == FR_OK)
	{
		SDReadingReady = 1u;
		return 1u;
	}

	(void)f_mount(NULL, "", 0);
	HAL_Delay(20);
	FatFsResult = f_mount(&SdFatFs, "", 1);
	if(FatFsResult == FR_OK)
	{
		SDReadingReady = 1u;
		return 1u;
	}

	SDReadingReady = 0u;
	SendFatFsError("SD_ERROR", "mount", context, FatFsResult);
	return 0u;
}

static uint8_t SdOpenFile(const char *error_prefix, const char *operation, const char *file_name, BYTE mode)
{
	if(SdEnsureMounted(file_name) == 0u)
	{
		return 0u;
	}

	FatFsResult = f_open(&SdCardFile, file_name, mode);
	if(FatFsResult == FR_OK)
	{
		return 1u;
	}

	(void)f_mount(NULL, "", 0);
	HAL_Delay(20);
	if(SdEnsureMounted(file_name) == 0u)
	{
		return 0u;
	}

	FatFsResult = f_open(&SdCardFile, file_name, mode);
	if(FatFsResult == FR_OK)
	{
		return 1u;
	}

	SendFatFsError(error_prefix, operation, file_name, FatFsResult);
	return 0u;
}

void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *ReceivedData)
{
	uint8_t tmp = 0u;
	uint8_t i = 0u;

	do
	{
		if(RB_Read(Buf, &tmp) != RB_OK)
		{
			ReceivedData[i] = 0u;
			return;
		}

		if(tmp == ENDLINE)
		{
			ReceivedData[i] = 0u;
		}
		else if((tmp != '\r') && (i < (PARSER_LINE_BUFFER_SIZE - 1u)))
		{
			ReceivedData[i++] = tmp;
		}
	} while(tmp != ENDLINE);
}

static void kp_change(LineFollower_t *LF)
{
	if(ParseFloatValue(&LF->Kp))
	{
		RobotSettings_RequestSave();
	}
}

static void kd_change(LineFollower_t *LF)
{
	if(ParseFloatValue(&LF->Kd))
	{
		RobotSettings_RequestSave();
	}
}

static void Base_speed_change(LineFollower_t *LF)
{
	float speed = 0.0f;
	if(ParseFloatValue(&speed))
	{
		LF->Base_speed_R = (int)speed;
		LF->Base_speed_L = (int)speed;
		RobotSettings_RequestSave();
	}
}

static void Max_speed_change(LineFollower_t *LF)
{
	float speed = 0.0f;
	if(ParseFloatValue(&speed))
	{
		LF->Max_speed_R = (int)speed;
		LF->Max_speed_L = (int)speed;
		RobotSettings_RequestSave();
	}
}

static void Sensor_treshold_change(LineFollower_t *LF)
{
	float threshold = 0.0f;
	if(ParseFloatValue(&threshold))
	{
		LF->treshold = (uint16_t)threshold;
		RobotSettings_RequestSave();
	}
}

static void Sharp_bend_speed_right_change(LineFollower_t *LF)
{
	float value = 0.0f;
	if(ParseFloatValue(&value))
	{
		LF->Sharp_bend_speed_right = (int)value;
		RobotSettings_RequestSave();
	}
}

static void Sharp_bend_speed_left_change(LineFollower_t *LF)
{
	float value = 0.0f;
	if(ParseFloatValue(&value))
	{
		LF->Sharp_bend_speed_left = (int)value;
		RobotSettings_RequestSave();
	}
}

static void Bend_speed_right_change(LineFollower_t *LF)
{
	float value = 0.0f;
	if(ParseFloatValue(&value))
	{
		LF->Bend_speed_right = (int)value;
		RobotSettings_RequestSave();
	}
}

static void Bend_speed_left_change(LineFollower_t *LF)
{
	float value = 0.0f;
	if(ParseFloatValue(&value))
	{
		LF->Bend_speed_left = (int)value;
		RobotSettings_RequestSave();
	}
}

static void Map_p_change(void)
{
	if(ParseFloatValue(&map.p))
	{
		RobotSettings_RequestSave();
	}
}

static void Map_i_change(void)
{
	if(ParseFloatValue(&map.i))
	{
		RobotSettings_RequestSave();
	}
}

static void Map_d_change(void)
{
	if(ParseFloatValue(&map.d))
	{
		RobotSettings_RequestSave();
	}
}

static void Map_speed_change(void)
{
	if(ParseFloatValue(&map.DefaultPlaybackSpeed))
	{
		RobotSettings_RequestSave();
	}
}

static void Tire_clean_speed_change(void)
{
	float speed = 0.0f;
	if(ParseFloatValue(&speed))
	{
		TireCleanSpeed = ClampFloat(speed, 0.0f, 250.0f);
	}
}

static void StopRobot(LineFollower_t *LF)
{
	LF->LineFollowing = 0u;
	LF->PowerMode = Stop;
	map.Mapping = 0u;
	map.PlaybackActive = 0u;
	TireCleaningActive = 0u;
	ManualDriveActive = 0u;

	motor_control(0.0, 0.0);
	HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);

	if(SdFileOpen != 0u)
	{
		f_close(&SdCardFile);
		SdFileOpen = 0u;
	}

	LF->battery_voltage = ((float)LF->Adc_Value * 8.3f) / 3831.0f;

	char buffer[160];
	snprintf(buffer, sizeof(buffer),
			 "Stop\r\nADC_Value = %u\r\nBattery = %0.2f V\r\nAverageSpeed = %0.2f m/s\r\nMaxSpeed = %0.2f m/s\r\n",
			 LF->Adc_Value, LF->battery_voltage, LF->AverageSpeed, LF->MaxSpeed);
	UartSend(buffer);

	LF->AverageSpeed = 0.0f;
	LF->MaxSpeed = 0.0f;
	LF->AverageSpeedNum = 0.0f;
	LF->AverageSpeedSum = 0.0f;
}

static void PrepareStoppedStart(LineFollower_t *LF)
{
	LF->LineFollowing = 0u;
	LF->PowerMode = Stop;
	map.Mapping = 0u;
	map.PlaybackActive = 0u;
	TelemetryMode = TELEMETRY_OFF;
	TireCleaningActive = 0u;
	ManualDriveActive = 0u;

	motor_control(0.0, 0.0);
	HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);

	if(SdFileOpen != 0u)
	{
		f_close(&SdCardFile);
		SdFileOpen = 0u;
	}

	LF->AverageSpeed = 0.0f;
	LF->MaxSpeed = 0.0f;
	LF->AverageSpeedNum = 0.0f;
	LF->AverageSpeedSum = 0.0f;
}

static void StartRobot(LineFollower_t *LF)
{
	char buffer[160];
	float battery_percentage;

	LF->battery_voltage = ((float)LF->Adc_Value * 8.3f) / 3831.0f;
	battery_percentage = (LF->battery_voltage / 8.24f) * 100.0f;

	if(LF->battery_voltage < 7.0f)
	{
		UartSend("! Low Battery !\r\n");
	}

	LF->Speed_level = ((200.0f - battery_percentage) / 100.0f) - LF->Speed_offset;
	if(LF->Speed_level < 1.0f)
	{
		LF->Speed_level = 1.0f;
	}

	Map_Reset(&map);
	Robot_ResetOdometry();
	LF->Yaw = 0.0f;
	LF->Orientation = 0.0f;
	LF->Last_error = 0.0f;
	LF->P = 0.0f;
	LF->D = 0.0f;
	LF->Error_P = 0.0f;
	LF->Error_D = 0.0f;
	LF->Last_idle = 0;
	LF->LastEndTimer = HAL_GetTick();
	LF->UnMappingDone = 0u;

	if(LF->state == Mapping)
	{
		if(SdOpenFile("MAP_ERROR", "open_write", ROBOT_MAP_OUTPUT_FILE, FA_WRITE | FA_CREATE_ALWAYS) == 0u)
		{
			return;
		}
		SdFileOpen = 1u;
		map.Mapping = 1u;
	}
	else if(LF->state == UnMapping)
	{
		if(SdOpenFile("MAP_ERROR", "open_playback", ROBOT_MAP_INPUT_FILE, FA_READ) == 0u)
		{
			return;
		}
		SdFileOpen = 1u;
		if(Map_BeginPlayback(&map) == 0u)
		{
			UartSend("map.txt is empty or invalid\r\n");
			f_close(&SdCardFile);
			SdFileOpen = 0u;
			return;
		}
	}

	snprintf(buffer, sizeof(buffer), "Start\r\nBattery = %0.2f V\r\nSpeed_level = %0.2f\r\n",
			 LF->battery_voltage, LF->Speed_level);
	UartSend(buffer);

	HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
	LF->LineFollowing = 1u;
	LF->PowerMode = Start;
}

static void StartWithState(LineFollower_t *LF, uint8_t state)
{
	PrepareStoppedStart(LF);
	LF->state = state;
	if(state == Mapping)
	{
		UartSend("STARTING,mapping\r\n");
	}
	else if(state == UnMapping)
	{
		UartSend("STARTING,playback\r\n");
	}
	else
	{
		UartSend("STARTING,normal\r\n");
	}
	StartRobot(LF);
}

static void SetState(LineFollower_t *LF, uint8_t state)
{
	if(LF->PowerMode == Start)
	{
		UartSend("Stop robot before changing state\r\n");
		return;
	}

	LF->state = state;
	if(state == Mapping)
	{
		UartSend("State: Mapping\r\n");
	}
	else if(state == UnMapping)
	{
		UartSend("State: UnMapping\r\n");
	}
	else
	{
		UartSend("State: PID\r\n");
	}
}

static void SetDrivePreset(LineFollower_t *LF,
						   int base,
						   int max,
						   int sharp_right,
						   int sharp_left,
						   int bend_right,
						   int bend_left,
						   float kp,
						   float kd)
{
	LF->Base_speed_R = base;
	LF->Base_speed_L = base;
	LF->Max_speed_R = max;
	LF->Max_speed_L = max;
	LF->Sharp_bend_speed_right = sharp_right;
	LF->Sharp_bend_speed_left = sharp_left;
	LF->Bend_speed_right = bend_right;
	LF->Bend_speed_left = bend_left;
	LF->Kp = kp;
	LF->Kd = kd;
	RobotSettings_RequestSave();
}

void App_Controll(char RxData, LineFollower_t *LineFollower)
{
	if(RxData == 'N')
	{
		StopRobot(LineFollower);
	}
	else if((RxData == 'Y') || (RxData == 'C'))
	{
		if(LineFollower->PowerMode != Start)
		{
			StartRobot(LineFollower);
		}
		else
		{
			UartSend("Already started\r\n");
		}
	}
	else if(RxData == 'P')
	{
		SetState(LineFollower, PidFollowing);
	}
	else if(RxData == 'M')
	{
		SetState(LineFollower, Mapping);
	}
	else if(RxData == 'U')
	{
		SetState(LineFollower, UnMapping);
	}
	else if(RxData == 'a')
	{
		SetDrivePreset(LineFollower, 85, 165, -40, 90, -50, 80, 0.015f, 0.085f);
	}
	else if(RxData == 'd')
	{
		SetDrivePreset(LineFollower, 95, 150, -30, 100, -40, 80, 0.015f, 0.08f);
	}
	else if(RxData == 'b')
	{
		SetDrivePreset(LineFollower, 102, 145, -30, 100, -40, 80, 0.015f, 0.08f);
	}
	else if(RxData == 'e')
	{
		SetDrivePreset(LineFollower, 105, 152, -30, 90, -40, 80, 0.015f, 0.08f);
	}
	else if(RxData == 'c')
	{
		SetDrivePreset(LineFollower, 100, 165, -20, 110, -40, 80, 0.015f, 0.08f);
	}
	else if(RxData == 'f')
	{
		SetDrivePreset(LineFollower, 115, 175, -20, 110, -40, 80, 0.015f, 0.08f);
	}
	else if(RxData == 'i')
	{
		SetDrivePreset(LineFollower, 135, 135, -60, 70, -60, 110, 0.015f, 0.2f);
	}
	else if(RxData == 'j')
	{
		SetDrivePreset(LineFollower, 140, 140, -60, 70, -60, 110, 0.015f, 0.2f);
	}
	else if(RxData == 'k')
	{
		SetDrivePreset(LineFollower, 145, 145, -60, 70, -60, 110, 0.015f, 0.22f);
	}
	else if(RxData == 'l')
	{
		SetDrivePreset(LineFollower, 150, 150, -55, 75, -60, 120, 0.015f, 0.23f);
	}
	else if(RxData == 'm')
	{
		SetDrivePreset(LineFollower, 155, 155, -70, 85, -50, 30, 0.015f, 0.265f);
	}
	else if(RxData == 'n')
	{
		SetDrivePreset(LineFollower, 160, 160, -70, 85, -50, 30, 0.015f, 0.265f);
	}
	else if(RxData == 'h')
	{
		SetDrivePreset(LineFollower, 55, 80, -70, 70, -50, 80, 0.015f, 0.07f);
	}
	else if(RxData == 'o')
	{
		SetDrivePreset(LineFollower, 143, 182, -76, 90, -50, 100, 0.02f, 350.0f);
	}
	else if(RxData == 'u')
	{
		SetDrivePreset(LineFollower, 153, 187, -76, 90, -50, 100, 0.02f, 350.0f);
	}
	else if(RxData == 'p')
	{
		SetDrivePreset(LineFollower, 143, 182, -76, 90, -40, 110, 0.02f, 350.0f);
	}
	else if(RxData == 'r')
	{
		SetDrivePreset(LineFollower, 153, 187, -76, 90, -40, 110, 0.02f, 350.0f);
	}
}

static void Mode_change(LineFollower_t *LF)
{
	char *parse_pointer = NextValue();
	if((parse_pointer != NULL) && (strlen(parse_pointer) > 0u))
	{
		App_Controll(parse_pointer[0], LF);
	}
}

static void State_change(LineFollower_t *LF)
{
	char *parse_pointer = NextValue();
	if(parse_pointer == NULL)
	{
		return;
	}

	if((!strcmp(parse_pointer, "PID")) || (!strcmp(parse_pointer, "Pid")) || (!strcmp(parse_pointer, "P")))
	{
		SetState(LF, PidFollowing);
	}
	else if((!strcmp(parse_pointer, "Mapping")) || (!strcmp(parse_pointer, "M")))
	{
		SetState(LF, Mapping);
	}
	else if((!strcmp(parse_pointer, "UnMapping")) || (!strcmp(parse_pointer, "U")))
	{
		SetState(LF, UnMapping);
	}
}

static void Start_command(LineFollower_t *LF, uint8_t state)
{
	(void)NextValue();
	StartWithState(LF, state);
}

static void Telemetry_change(void)
{
	char *parse_pointer = NextValue();
	if(parse_pointer == NULL)
	{
		return;
	}

	if((!strcmp(parse_pointer, "odom")) || (!strcmp(parse_pointer, "ODOM")) || (!strcmp(parse_pointer, "1")))
	{
		TelemetryMode = TELEMETRY_ODOM;
		UartSend("TELEMETRY,odom\r\n");
	}
	else if((!strcmp(parse_pointer, "debug")) || (!strcmp(parse_pointer, "DEBUG")) || (!strcmp(parse_pointer, "2")))
	{
		TelemetryMode = TELEMETRY_DEBUG;
		UartSend("TELEMETRY,debug\r\n");
	}
	else
	{
		TelemetryMode = TELEMETRY_OFF;
		UartSend("TELEMETRY,off\r\n");
	}
}

static void Manual_drive_change(LineFollower_t *LF)
{
	float left_speed = 0.0f;
	float right_speed = 0.0f;
	if(!ParseFloatValue(&left_speed))
	{
		return;
	}
	if(!ParseFloatValue(&right_speed))
	{
		right_speed = left_speed;
	}

	if((fabsf(left_speed) < 0.5f) && (fabsf(right_speed) < 0.5f))
	{
		ManualDriveActive = 0u;
		motor_control(0.0, 0.0);
		HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);
		return;
	}

	if(LF->PowerMode == Start)
	{
		UartSend("MANUAL_ERROR,robot_running\r\n");
		return;
	}

	if(SdFileOpen != 0u)
	{
		f_close(&SdCardFile);
		SdFileOpen = 0u;
	}

	TireCleaningActive = 0u;
	ManualDriveActive = 1u;
	ManualDriveLastTick = HAL_GetTick();
	LF->LineFollowing = 0u;
	LF->PowerMode = Stop;
	map.Mapping = 0u;
	map.PlaybackActive = 0u;
	TelemetryMode = TELEMETRY_OFF;

	left_speed = ClampFloat(left_speed, -250.0f, 250.0f);
	right_speed = ClampFloat(right_speed, -250.0f, 250.0f);
	HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_SET);
	motor_control(right_speed, left_speed);
}

static void Tire_clean_change(LineFollower_t *LF)
{
	float enabled = 0.0f;
	if(!ParseFloatValue(&enabled))
	{
		return;
	}

	if(enabled > 0.0f)
	{
		if(LF->PowerMode == Start)
		{
			UartSend("CLEAN_ERROR,robot_running\r\n");
			return;
		}

		if(SdFileOpen != 0u)
		{
			f_close(&SdCardFile);
			SdFileOpen = 0u;
		}

		LF->LineFollowing = 0u;
		LF->PowerMode = Stop;
		map.Mapping = 0u;
		map.PlaybackActive = 0u;
		TelemetryMode = TELEMETRY_OFF;
		ManualDriveActive = 0u;
		TireCleaningActive = 1u;
		HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_SET);
		motor_control(TireCleanSpeed, TireCleanSpeed);
		UartSend("CLEAN,start\r\n");
	}
	else
	{
		TireCleaningActive = 0u;
		motor_control(0.0, 0.0);
		HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);
		UartSend("CLEAN,stop\r\n");
	}
}

void Parser_ServiceManualDriveTimeout(LineFollower_t *LF)
{
	if((ManualDriveActive != 0u) && ((HAL_GetTick() - ManualDriveLastTick) > ROBOT_MANUAL_TIMEOUT_MS))
	{
		ManualDriveActive = 0u;
		motor_control(0.0, 0.0);
		HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);
	}
	(void)LF;
}

uint8_t Parser_ServicePendingBluetoothName(void)
{
	char name[ROBOT_BT_NAME_MAX_LEN + 1u];
	uint32_t baud = 0u;

	if(BluetoothPendingNameRead(name, sizeof(name)) == 0u)
	{
		return 0u;
	}

	if(BluetoothApplyName(name, &baud) != 0u)
	{
		(void)f_unlink(ROBOT_BT_NAME_FILE);
		char tx[72];
		snprintf(tx, sizeof(tx), "BT_NAME_OK,%s,%lu\r\n", name, (unsigned long)baud);
		UartSend(tx);
		return 1u;
	}

	UartSend("BT_NAME_ERROR,at_no_response\r\n");
	return 0u;
}

static void Bluetooth_name_store(void)
{
	char name[ROBOT_BT_NAME_MAX_LEN + 1u];
	if(ReadBluetoothNameArgument(name, sizeof(name)) == 0u)
	{
		return;
	}

	if(BluetoothPendingNameWrite(name) != 0u)
	{
		char tx[72];
		snprintf(tx, sizeof(tx), "BT_NAME_PENDING,%s\r\n", name);
		UartSend(tx);
	}
}

static void Bluetooth_name_now(LineFollower_t *LF)
{
	char name[ROBOT_BT_NAME_MAX_LEN + 1u];
	uint32_t baud = 0u;

	if(LF->PowerMode == Start)
	{
		UartSend("BT_NAME_ERROR,stop_robot_first\r\n");
		return;
	}

	if(ReadBluetoothNameArgument(name, sizeof(name)) == 0u)
	{
		return;
	}

	if(SdFileOpen != 0u)
	{
		f_close(&SdCardFile);
		SdFileOpen = 0u;
	}

	LF->LineFollowing = 0u;
	LF->PowerMode = Stop;
	map.Mapping = 0u;
	map.PlaybackActive = 0u;
	TelemetryMode = TELEMETRY_OFF;
	TireCleaningActive = 0u;
	ManualDriveActive = 0u;
	motor_control(0.0, 0.0);
	HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);

	if(BluetoothApplyName(name, &baud) != 0u)
	{
		(void)f_unlink(ROBOT_BT_NAME_FILE);
		char tx[72];
		snprintf(tx, sizeof(tx), "BT_NAME_OK,%s,%lu\r\n", name, (unsigned long)baud);
		UartSend(tx);
	}
	else
	{
		UartSend("BT_NAME_ERROR,at_no_response\r\n");
	}
}

void Parser_FinishMappingAutoClosed(LineFollower_t *LF)
{
	LF->LineFollowing = 0u;
	LF->PowerMode = Stop;
	map.Mapping = 0u;
	map.PlaybackActive = 0u;
	TireCleaningActive = 0u;
	ManualDriveActive = 0u;

	motor_control(0.0, 0.0);
	HAL_GPIO_WritePin(Motor_STBY_GPIO_Port, Motor_STBY_Pin, GPIO_PIN_RESET);

	if(SdFileOpen != 0u)
	{
		f_close(&SdCardFile);
		SdFileOpen = 0u;
	}

	UartSend("MAP_AUTO_CLOSED,start_radius_3cm\r\n");
}

static const char *MapFileFromKind(const char *kind, const char **normalized_kind)
{
	if((kind != NULL) &&
	   ((!strcmp(kind, "optimized")) || (!strcmp(kind, "map")) || (!strcmp(kind, "map.txt"))))
	{
		*normalized_kind = "optimized";
		return ROBOT_MAP_INPUT_FILE;
	}

	*normalized_kind = "recorded";
	return ROBOT_MAP_OUTPUT_FILE;
}

static uint8_t ReadTransferLine(char *line, size_t line_size)
{
	UINT len = 0u;
	uint8_t i = 0u;
	uint8_t sample = 0u;

	while(i < (uint8_t)(line_size - 1u))
	{
		if((f_read(&SdCardFile, &sample, 1, &len) != FR_OK) || (len == 0u))
		{
			break;
		}

		if(sample == '\r')
		{
			continue;
		}

		if(sample == '\n')
		{
			break;
		}

		line[i++] = (char)sample;
	}

	line[i] = '\0';
	return ((i > 0u) || (len != 0u)) ? 1u : 0u;
}

static void SendMapDump(LineFollower_t *LF)
{
	if(LF->PowerMode == Start)
	{
		UartSend("MAP_ERROR,stop_robot_first\r\n");
		return;
	}

	char *requested_kind = NextValue();
	const char *dump_kind = "recorded";
	const char *file_name = MapFileFromKind(requested_kind, &dump_kind);

	if(SdFileOpen != 0u)
	{
		f_close(&SdCardFile);
		SdFileOpen = 0u;
	}

	if(SdOpenFile("MAP_ERROR", "open_read", file_name, FA_READ) == 0u)
	{
		return;
	}

	char tx[160];
	char line[112];
	uint16_t count = 0u;
	snprintf(tx, sizeof(tx), "MAP_BEGIN,%s\r\n", dump_kind);
	UartSend(tx);

	while(ReadTransferLine(line, sizeof(line)) != 0u)
	{
		snprintf(tx, sizeof(tx), "MAP,%s\r\n", line);
		UartSend(tx);
		count++;
		HAL_Delay(2);
	}

	f_close(&SdCardFile);
	snprintf(tx, sizeof(tx), "MAP_END,%u\r\n", count);
	UartSend(tx);
}

static void MapUploadBegin(LineFollower_t *LF)
{
	if(LF->PowerMode == Start)
	{
		UartSend("UPLOAD_ERROR,stop_robot_first\r\n");
		return;
	}

	float expected_count = 0.0f;
	(void)ParseFloatValue(&expected_count);

	if(SdFileOpen != 0u)
	{
		f_close(&SdCardFile);
		SdFileOpen = 0u;
	}

	if(SdOpenFile("UPLOAD_ERROR", "open_write", ROBOT_MAP_INPUT_FILE, FA_WRITE | FA_CREATE_ALWAYS) == 0u)
	{
		return;
	}

	SdFileOpen = 1u;
	MapUploadPointCount = 0u;
	MapUploadExpectedCount = (uint16_t)expected_count;

	char tx[80];
	snprintf(tx, sizeof(tx), "UPLOAD_READY,%u\r\n", MapUploadExpectedCount);
	UartSend(tx);
}

static void MapUploadPoint(void)
{
	if(SdFileOpen == 0u)
	{
		UartSend("UPLOAD_ERROR,no_file\r\n");
		return;
	}

	float x = 0.0f;
	float y = 0.0f;
	float speed = map.DefaultPlaybackSpeed;
	if((!ParseFloatValue(&x)) || (!ParseFloatValue(&y)))
	{
		UartSend("UPLOAD_ERROR,bad_point\r\n");
		return;
	}
	(void)ParseFloatValue(&speed);

	char line[64];
	snprintf(line, sizeof(line), "%0.3f\t%0.3f\t%0.3f\n", x, y, speed);
	f_puts((TCHAR *)line, &SdCardFile);
	MapUploadPointCount++;

	if((MapUploadPointCount % 50u) == 0u)
	{
		char tx[80];
		snprintf(tx, sizeof(tx), "UPLOAD_PROGRESS,%u,%u\r\n", MapUploadPointCount, MapUploadExpectedCount);
		UartSend(tx);
	}
}

static void MapUploadEnd(void)
{
	if(SdFileOpen != 0u)
	{
		f_close(&SdCardFile);
		SdFileOpen = 0u;
	}

	char tx[80];
	snprintf(tx, sizeof(tx), "UPLOAD_DONE,%u,%u\r\n", MapUploadPointCount, MapUploadExpectedCount);
	UartSend(tx);
}

static void SaveSettingsCommand(LineFollower_t *LF)
{
	char tx[64];

	if(LF->PowerMode == Start)
	{
		RobotSettings_RequestSave();
		UartSend("SETTINGS_SAVE,pending_until_stop\r\n");
		return;
	}

	RobotSettingsStatus_t status = RobotSettings_Save(LF, &map);
	snprintf(tx, sizeof(tx), "SETTINGS_SAVE,%s\r\n", SettingsStatusText(status));
	UartSend(tx);
}

static void LoadSettingsCommand(LineFollower_t *LF)
{
	char tx[64];

	if(LF->PowerMode == Start)
	{
		UartSend("SETTINGS_LOAD,stop_robot_first\r\n");
		return;
	}

	RobotSettingsStatus_t status = RobotSettings_Load(LF, &map);
	snprintf(tx, sizeof(tx), "SETTINGS_LOAD,%s\r\n", SettingsStatusText(status));
	UartSend(tx);
}

void Parser_Parse(uint8_t *ReceivedData, LineFollower_t *LineFollower)
{
	char *parse_pointer = strtok((char*)ReceivedData, "=");
	if(parse_pointer == NULL)
	{
		return;
	}

	if(!strcmp("Kp", parse_pointer))
	{
		kp_change(LineFollower);
	}
	else if(!strcmp("Kd", parse_pointer))
	{
		kd_change(LineFollower);
	}
	else if(!strcmp("Base_speed", parse_pointer))
	{
		Base_speed_change(LineFollower);
	}
	else if(!strcmp("Max_speed", parse_pointer))
	{
		Max_speed_change(LineFollower);
	}
	else if(!strcmp("Sharp_bend_speed_right", parse_pointer))
	{
		Sharp_bend_speed_right_change(LineFollower);
	}
	else if(!strcmp("Sharp_bend_speed_left", parse_pointer))
	{
		Sharp_bend_speed_left_change(LineFollower);
	}
	else if(!strcmp("Bend_speed_right", parse_pointer))
	{
		Bend_speed_right_change(LineFollower);
	}
	else if(!strcmp("Bend_speed_left", parse_pointer))
	{
		Bend_speed_left_change(LineFollower);
	}
	else if(!strcmp("Mode", parse_pointer))
	{
		Mode_change(LineFollower);
	}
	else if(!strcmp("State", parse_pointer))
	{
		State_change(LineFollower);
	}
	else if(!strcmp("StartNormal", parse_pointer))
	{
		Start_command(LineFollower, PidFollowing);
	}
	else if(!strcmp("StartMapping", parse_pointer))
	{
		Start_command(LineFollower, Mapping);
	}
	else if(!strcmp("StartPlayback", parse_pointer))
	{
		Start_command(LineFollower, UnMapping);
	}
	else if(!strcmp("MapP", parse_pointer))
	{
		Map_p_change();
	}
	else if(!strcmp("Treshold", parse_pointer))
	{
		Sensor_treshold_change(LineFollower);
	}
	else if(!strcmp("MapI", parse_pointer))
	{
		Map_i_change();
	}
	else if(!strcmp("MapD", parse_pointer))
	{
		Map_d_change();
	}
	else if(!strcmp("MapSpeed", parse_pointer))
	{
		Map_speed_change();
	}
	else if((!strcmp("CleanSpeed", parse_pointer)) || (!strcmp("TireCleanSpeed", parse_pointer)))
	{
		Tire_clean_speed_change();
	}
	else if((!strcmp("Clean", parse_pointer)) || (!strcmp("TireClean", parse_pointer)))
	{
		Tire_clean_change(LineFollower);
	}
	else if((!strcmp("Manual", parse_pointer)) || (!strcmp("Joystick", parse_pointer)))
	{
		Manual_drive_change(LineFollower);
	}
	else if(!strcmp("Telemetry", parse_pointer))
	{
		Telemetry_change();
	}
	else if(!strcmp("Debug", parse_pointer))
	{
		TelemetryMode = TELEMETRY_DEBUG;
		UartSend("TELEMETRY,debug\r\n");
	}
	else if((!strcmp("BtName", parse_pointer)) || (!strcmp("BTName", parse_pointer)) ||
			(!strcmp("BluetoothName", parse_pointer)))
	{
		Bluetooth_name_store();
	}
	else if((!strcmp("BtNameNow", parse_pointer)) || (!strcmp("BTNameNow", parse_pointer)) ||
			(!strcmp("BluetoothNameNow", parse_pointer)))
	{
		Bluetooth_name_now(LineFollower);
	}
	else if(!strcmp("MapDump", parse_pointer))
	{
		SendMapDump(LineFollower);
	}
	else if(!strcmp("MapUploadBegin", parse_pointer))
	{
		MapUploadBegin(LineFollower);
	}
	else if(!strcmp("MapPoint", parse_pointer))
	{
		MapUploadPoint();
	}
	else if(!strcmp("MapUploadEnd", parse_pointer))
	{
		MapUploadEnd();
	}
	else if((!strcmp("SaveSettings", parse_pointer)) || (!strcmp("SettingsSave", parse_pointer)))
	{
		SaveSettingsCommand(LineFollower);
	}
	else if((!strcmp("LoadSettings", parse_pointer)) || (!strcmp("SettingsLoad", parse_pointer)))
	{
		LoadSettingsCommand(LineFollower);
	}
}

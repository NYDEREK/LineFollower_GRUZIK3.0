/*
 * map.c
 *
 * Route mapping and playback for the renewed GRUZIK3.0 platform.
 */

#include "map.h"
#include "Line_Follower.h"
#include "app_fatfs.h"
#include "robot_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern LineFollower_t GRUZIK;
extern FRESULT FatFsResult;
extern FIL SdCardFile;

static uint8_t ReadMapLine(Map_t *map);
static void WriteMapRecord(float x_ri, float y_ri, float x_pc, float y_pc, float speed_mps, float error_p, float error_d);

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

static void UpdateLineError(LineFollower_t *LF)
{
	int position = QTR8_read();
	float error = 6500.0f - (float)position;

	LF->P = error;
	LF->Error_P = error;
	LF->D = error - LF->Last_error;
	LF->Error_D = LF->D;
	LF->Last_error = error;
}

static float LineConfidence(const LineFollower_t *LF)
{
	if(LF->actives == 0)
	{
		return 0.0f;
	}

	float center_error = fabsf(6500.0f - (float)LF->SensorPosition);
	float confidence = 1.0f - ClampFloat((center_error - 1800.0f) / 3600.0f, 0.0f, 1.0f);

	if(LF->actives > 7)
	{
		confidence *= 0.45f;
	}

	return confidence;
}

static uint8_t AdvanceMapTarget(Map_t *map)
{
	map->PreviousSetX = map->SetX;
	map->PreviousSetY = map->SetY;
	return ReadMapLine(map);
}

static uint8_t ShouldAdvanceTarget(const Map_t *map, const Pose2D_t *pose, float distance_to_target)
{
	if(distance_to_target < ROBOT_MAP_TARGET_RADIUS_M)
	{
		return 1u;
	}
	if(map->TargetValid == 0u)
	{
		return 0u;
	}

	float segment_x = map->SetX - map->PreviousSetX;
	float segment_y = map->SetY - map->PreviousSetY;
	float segment_len_sq = (segment_x * segment_x) + (segment_y * segment_y);
	if(segment_len_sq < 0.000001f)
	{
		return 0u;
	}

	float pose_x = pose->x_m - map->PreviousSetX;
	float pose_y = pose->y_m - map->PreviousSetY;
	float projection = ((pose_x * segment_x) + (pose_y * segment_y)) / segment_len_sq;
	float lateral_error = fabsf((pose_x * segment_y) - (pose_y * segment_x)) / sqrtf(segment_len_sq);

	if((projection > 1.02f) && (lateral_error < ROBOT_MAP_ADVANCE_LATERAL_M))
	{
		return 1u;
	}
	if(projection > 1.30f)
	{
		return 1u;
	}

	return 0u;
}

static uint8_t ReadMapLine(Map_t *map)
{
	if(FatFsResult != FR_OK)
	{
		return 0u;
	}

	while(1)
	{
		UINT len = 0u;
		uint8_t i = 0u;
		uint8_t sample = 0u;
		char line[96];

		while(i < (uint8_t)(sizeof(line) - 1u))
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

		if((i == 0u) && (len == 0u))
		{
			return 0u;
		}

		char *token = strtok(line, " \t");
		if(token == NULL)
		{
			continue;
		}
		map->SetX = strtof(token, NULL);

		token = strtok(NULL, " \t");
		if(token == NULL)
		{
			continue;
		}
		map->SetY = strtof(token, NULL);

		token = strtok(NULL, " \t");
		if(token != NULL)
		{
			map->SetSpeed = ClampFloat(strtof(token, NULL), -ROBOT_PWM_MAX, ROBOT_PWM_MAX);
		}
		else
		{
			map->SetSpeed = map->DefaultPlaybackSpeed;
		}

		return 1u;
	}
}

static void WriteMapRecord(float x_ri, float y_ri, float x_pc, float y_pc, float speed_mps, float error_p, float error_d)
{
	char buffer[128];
	snprintf(buffer, sizeof(buffer), "%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\n",
			 x_ri, y_ri, x_pc, y_pc, speed_mps, error_p, error_d);
	f_puts((TCHAR *)buffer, &SdCardFile);
}

void Map_Reset(Map_t *map)
{
	float p = map->p;
	float i = map->i;
	float d = map->d;
	float default_speed = map->DefaultPlaybackSpeed;

	memset(map, 0, sizeof(*map));
	map->p = p;
	map->i = i;
	map->d = d;
	map->DefaultPlaybackSpeed = (default_speed > 0.0f) ? default_speed : 80.0f;
}

uint8_t Map_BeginPlayback(Map_t *map)
{
	map->PlaybackActive = 0u;
	map->ErrorSum = 0.0f;
	map->LastError = 0.0f;
	map->FilteredErrorDerivative = 0.0f;
	map->PreviousSetX = 0.0f;
	map->PreviousSetY = 0.0f;
	map->TargetValid = 0u;

	if(ReadMapLine(map) == 0u)
	{
		return 0u;
	}

	map->TargetValid = 1u;
	map->PlaybackActive = 1u;
	return 1u;
}

void Map_SyncPose(Map_t *map, const Pose2D_t *pose, float delta_distance_m)
{
	map->PreviousXri = map->Xri;
	map->PreviousYri = map->Yri;
	map->PreviousOri = map->Ori;
	map->PreviousPci[0] = map->Pci[0];
	map->PreviousPci[1] = map->Pci[1];

	map->Xri = pose->x_m;
	map->Yri = pose->y_m;
	map->Ori = pose->yaw_rad;
	map->Pci[0] = map->Xri + (ROBOT_SENSOR_OFFSET_M * cosf(map->Ori));
	map->Pci[1] = map->Yri + (ROBOT_SENSOR_OFFSET_M * sinf(map->Ori));
	map->Ti = delta_distance_m;
	map->Qri[0] = map->Xri;
	map->Qri[1] = map->Yri;
	map->Qri[2] = map->Ori;
}

uint8_t MapUpdate(Map_t *map, const Pose2D_t *pose, float speed_mps, float error_p, float error_d)
{
	if(map->Mapping == 0u)
	{
		return 1u;
	}

	map->Xri = pose->x_m;
	map->Yri = pose->y_m;
	map->Ori = pose->yaw_rad;
	map->Pci[0] = map->Xri + (ROBOT_SENSOR_OFFSET_M * cosf(map->Ori));
	map->Pci[1] = map->Yri + (ROBOT_SENSOR_OFFSET_M * sinf(map->Ori));
	map->Qri[0] = map->Xri;
	map->Qri[1] = map->Yri;
	map->Qri[2] = map->Ori;

	if(map->StartValid == 0u)
	{
		map->StartValid = 1u;
		map->StartXri = map->Xri;
		map->StartYri = map->Yri;
		map->StartOri = map->Ori;
		map->StartPci[0] = map->Pci[0];
		map->StartPci[1] = map->Pci[1];
		map->PreviousXri = map->Xri;
		map->PreviousYri = map->Yri;
		map->PreviousOri = map->Ori;
		map->PreviousPci[0] = map->Pci[0];
		map->PreviousPci[1] = map->Pci[1];
		map->Ai = map->Ori;
		map->PreviousAi = map->Ori;
		WriteMapRecord(map->Xri, map->Yri, map->Pci[0], map->Pci[1], speed_mps, error_p, error_d);
		map->RecordedPoints++;
		return 1u;
	}

	float dx = map->Pci[0] - map->PreviousPci[0];
	float dy = map->Pci[1] - map->PreviousPci[1];
	map->Ti = sqrtf((dx * dx) + (dy * dy));
	map->Si += map->Ti;

	if(map->Ti > 0.000001f)
	{
		map->PreviousAi = map->Ai;
		map->Ai = atan2f(dy, dx);
		map->Ki = Odometry_NormalizeAngle(map->Ai - map->PreviousAi) / map->Ti;
	}

	if(map->loopNumber >= ROBOT_MAP_RECORD_EVERY_TICKS)
	{
		map->loopNumber = 0u;
		WriteMapRecord(map->Xri, map->Yri, map->Pci[0], map->Pci[1], speed_mps, error_p, error_d);
		map->RecordedPoints++;
	}
	map->loopNumber++;

	float close_dx = map->Xri - map->StartXri;
	float close_dy = map->Yri - map->StartYri;
	float close_distance = sqrtf((close_dx * close_dx) + (close_dy * close_dy));
	if((map->Si >= ROBOT_MAP_CLOSE_MIN_DISTANCE_M) &&
	   (map->RecordedPoints >= ROBOT_MAP_CLOSE_MIN_POINTS) &&
	   (close_distance <= ROBOT_MAP_CLOSE_RADIUS_M))
	{
		WriteMapRecord(map->StartXri, map->StartYri, map->StartPci[0], map->StartPci[1], speed_mps, error_p, error_d);
		map->RecordedPoints++;
		map->AutoClosed = 1u;
		map->Mapping = 0u;
		return 0u;
	}

	return 1u;
}

uint8_t DriveOnMap(Map_t *map, const Pose2D_t *pose)
{
	if(map->PlaybackActive == 0u)
	{
		motor_control(0.0, 0.0);
		return 0u;
	}

	Map_SyncPose(map, pose, 0.0f);

	float dx = map->SetX - map->Xri;
	float dy = map->SetY - map->Yri;
	float distance_to_target = sqrtf((dx * dx) + (dy * dy));

	while(ShouldAdvanceTarget(map, pose, distance_to_target) != 0u)
	{
		if(AdvanceMapTarget(map) == 0u)
		{
			map->PlaybackActive = 0u;
			GRUZIK.DrivingOnMap = 0u;
			GRUZIK.UnMappingDone = 1u;
			GRUZIK.DoneUnMappingTimer = HAL_GetTick();
			GRUZIK.state = PidFollowing;
			return 0u;
		}

		dx = map->SetX - map->Xri;
		dy = map->SetY - map->Yri;
		distance_to_target = sqrtf((dx * dx) + (dy * dy));
	}

	float segment_x = map->SetX - map->PreviousSetX;
	float segment_y = map->SetY - map->PreviousSetY;
	float segment_len = sqrtf((segment_x * segment_x) + (segment_y * segment_y));
	float error = 0.0f;

	if(segment_len > 0.000001f)
	{
		map->SetRotation = atan2f(segment_y, segment_x);
		float pose_x = map->Xri - map->PreviousSetX;
		float pose_y = map->Yri - map->PreviousSetY;
		float cross_track = ((pose_x * segment_y) - (pose_y * segment_x)) / segment_len;
		cross_track = ClampFloat(cross_track, -ROBOT_PLAYBACK_CROSSTRACK_LIMIT_M, ROBOT_PLAYBACK_CROSSTRACK_LIMIT_M);
		float speed_ratio = ClampFloat(fabsf(map->SetSpeed) / ROBOT_PWM_MAX, 0.0f, 1.0f);
		float lookahead = ROBOT_PLAYBACK_LOOKAHEAD_BASE_M + (ROBOT_PLAYBACK_LOOKAHEAD_SPEED_M * speed_ratio);
		error = Odometry_NormalizeAngle((map->SetRotation - map->Ori) + atan2f(cross_track, lookahead));
	}
	else
	{
		map->SetRotation = atan2f(dy, dx);
		error = Odometry_NormalizeAngle(map->SetRotation - map->Ori);
	}

	map->ErrorSum = ClampFloat(map->ErrorSum + error, -3.0f, 3.0f);

	float error_derivative = Odometry_NormalizeAngle(error - map->LastError);
	map->LastError = error;
	map->FilteredErrorDerivative += ROBOT_PLAYBACK_D_FILTER_ALPHA * (error_derivative - map->FilteredErrorDerivative);

	float odom_correction = (map->p * error) + (map->i * map->ErrorSum) + (map->d * map->FilteredErrorDerivative);
	float odom_limit = ClampFloat(fabsf(map->SetSpeed) * 0.85f, 45.0f, ROBOT_PLAYBACK_ODOM_CORR_LIMIT_PWM);
	odom_correction = ClampFloat(odom_correction, -odom_limit, odom_limit);

	UpdateLineError(&GRUZIK);
	float line_correction = (GRUZIK.P * GRUZIK.Kp) + (GRUZIK.D * GRUZIK.Kd);
	line_correction = ClampFloat(line_correction,
								 -ROBOT_PLAYBACK_LINE_CORR_LIMIT_PWM,
								 ROBOT_PLAYBACK_LINE_CORR_LIMIT_PWM);

	float motor_correction = -odom_correction;
	float base_speed = map->SetSpeed;
	float turn_speed_scale = 1.0f - (ClampFloat(fabsf(error), 0.0f, 1.0f) * ROBOT_PLAYBACK_TURN_SPEED_REDUCTION);
	base_speed *= turn_speed_scale;

	float line_confidence = LineConfidence(&GRUZIK);
	if(line_confidence > 0.0f)
	{
		motor_correction += line_correction * ROBOT_PLAYBACK_LINE_WEIGHT * line_confidence;
	}
	if(GRUZIK.actives == 0)
	{
		base_speed *= ROBOT_PLAYBACK_LOST_LINE_SPEED_MUL;
	}

	motor_correction = ClampFloat(motor_correction,
								  -ROBOT_PLAYBACK_CORR_LIMIT_PWM,
								  ROBOT_PLAYBACK_CORR_LIMIT_PWM);

	float left_speed = ClampFloat(base_speed + motor_correction, -ROBOT_PWM_MAX, ROBOT_PWM_MAX);
	float right_speed = ClampFloat(base_speed - motor_correction, -ROBOT_PWM_MAX, ROBOT_PWM_MAX);
	motor_control(right_speed, left_speed);
	return 1u;
}

uint8_t MapUpdateV2(Map_t *map, const Pose2D_t *pose, float speed_mps, float error_p, float error_d)
{
	return MapUpdate(map, pose, speed_mps, error_p, error_d);
}

uint8_t SpeedUnMapping(Map_t *map, const Pose2D_t *pose)
{
	if(map->PlaybackActive == 0u)
	{
		return 0u;
	}

	Map_SyncPose(map, pose, 0.0f);

	float dx = map->SetX - map->Xri;
	float dy = map->SetY - map->Yri;
	float distance_to_target = sqrtf((dx * dx) + (dy * dy));
	while(ShouldAdvanceTarget(map, pose, distance_to_target) != 0u)
	{
		if(AdvanceMapTarget(map) == 0u)
		{
			map->PlaybackActive = 0u;
			GRUZIK.UnMappingDone = 1u;
			GRUZIK.DoneUnMappingTimer = HAL_GetTick();
			GRUZIK.state = PidFollowing;
			return 0u;
		}
		dx = map->SetX - map->Xri;
		dy = map->SetY - map->Yri;
		distance_to_target = sqrtf((dx * dx) + (dy * dy));
	}

	GRUZIK.Base_speed_L = (int)map->SetSpeed;
	GRUZIK.Base_speed_R = (int)map->SetSpeed;
	GRUZIK.Max_speed_L = (int)map->SetSpeed;
	GRUZIK.Max_speed_R = (int)map->SetSpeed;
	return 1u;
}

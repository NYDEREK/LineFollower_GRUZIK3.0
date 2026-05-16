/*
 * robot_settings.h
 *
 * Persistent tuning settings stored in the last Flash page.
 */

#ifndef INC_ROBOT_SETTINGS_H_
#define INC_ROBOT_SETTINGS_H_

#include "Line_Follower.h"
#include "map.h"

typedef enum
{
	ROBOT_SETTINGS_OK = 0,
	ROBOT_SETTINGS_EMPTY,
	ROBOT_SETTINGS_INVALID,
	ROBOT_SETTINGS_FLASH_ERROR
} RobotSettingsStatus_t;

RobotSettingsStatus_t RobotSettings_Load(LineFollower_t *LF, Map_t *map);
RobotSettingsStatus_t RobotSettings_Save(const LineFollower_t *LF, const Map_t *map);
void RobotSettings_RequestSave(void);
void RobotSettings_Service(const LineFollower_t *LF, const Map_t *map);
RobotSettingsStatus_t RobotSettings_LastStatus(void);

#endif /* INC_ROBOT_SETTINGS_H_ */

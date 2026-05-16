/*
 * robot_settings.c
 *
 * Persistent tuning settings stored in the last Flash page.
 */

#include "robot_settings.h"
#include "robot_config.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define ROBOT_SETTINGS_MAGIC       0x47525A33u
#define ROBOT_SETTINGS_VERSION     1u
#define ROBOT_SETTINGS_SAVE_DELAY  1200u

typedef struct
{
	uint32_t magic;
	uint32_t version;
	uint32_t size;
	float lf_kp;
	float lf_kd;
	float speed_offset;
	int32_t base_speed_r;
	int32_t base_speed_l;
	int32_t max_speed_r;
	int32_t max_speed_l;
	int32_t sharp_bend_speed_right;
	int32_t sharp_bend_speed_left;
	int32_t bend_speed_right;
	int32_t bend_speed_left;
	uint16_t threshold;
	uint16_t reserved0;
	float map_p;
	float map_i;
	float map_d;
	float map_default_speed;
	uint32_t checksum;
	uint32_t reserved1;
} RobotSettingsRecord_t;

static volatile uint8_t SettingsSaveDirty;
static uint32_t SettingsSaveRequestTick;
static RobotSettingsStatus_t SettingsLastStatus = ROBOT_SETTINGS_EMPTY;

static uint8_t SettingsFlashDualBank(void);

static uint32_t SettingsFlashAddress(void)
{
	uint32_t page_size = FLASH_PAGE_SIZE;
#if defined(FLASH_OPTR_DBANK)
	if(SettingsFlashDualBank() == 0u)
	{
		page_size = FLASH_PAGE_SIZE_128_BITS;
	}
#endif
	return FLASH_BASE + FLASH_SIZE - page_size;
}

static uint8_t SettingsFlashDualBank(void)
{
#if defined(FLASH_OPTR_DBANK)
	return ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0u) ? 1u : 0u;
#else
	return 0u;
#endif
}

static uint32_t SettingsFlashBank(uint32_t address)
{
#if defined(FLASH_OPTR_DBANK)
	if(SettingsFlashDualBank() != 0u)
	{
		return (address < (FLASH_BASE + FLASH_BANK_SIZE)) ? FLASH_BANK_1 : FLASH_BANK_2;
	}
#endif
	(void)address;
	return FLASH_BANK_1;
}

static uint32_t SettingsFlashPage(uint32_t address)
{
#if defined(FLASH_OPTR_DBANK)
	if(SettingsFlashDualBank() != 0u)
	{
		uint32_t bank_base = (SettingsFlashBank(address) == FLASH_BANK_2) ? (FLASH_BASE + FLASH_BANK_SIZE) : FLASH_BASE;
		return (address - bank_base) / FLASH_PAGE_SIZE;
	}
	return (address - FLASH_BASE) / FLASH_PAGE_SIZE_128_BITS;
#endif
	return (address - FLASH_BASE) / FLASH_PAGE_SIZE;
}

static uint32_t SettingsChecksum(const uint8_t *data, size_t len)
{
	uint32_t hash = 2166136261u;

	for(size_t i = 0u; i < len; i++)
	{
		hash ^= data[i];
		hash *= 16777619u;
	}

	return hash;
}

static void SettingsFillRecord(RobotSettingsRecord_t *record, const LineFollower_t *LF, const Map_t *map)
{
	memset(record, 0, sizeof(*record));
	record->magic = ROBOT_SETTINGS_MAGIC;
	record->version = ROBOT_SETTINGS_VERSION;
	record->size = sizeof(*record);
	record->lf_kp = LF->Kp;
	record->lf_kd = LF->Kd;
	record->speed_offset = LF->Speed_offset;
	record->base_speed_r = LF->Base_speed_R;
	record->base_speed_l = LF->Base_speed_L;
	record->max_speed_r = LF->Max_speed_R;
	record->max_speed_l = LF->Max_speed_L;
	record->sharp_bend_speed_right = LF->Sharp_bend_speed_right;
	record->sharp_bend_speed_left = LF->Sharp_bend_speed_left;
	record->bend_speed_right = LF->Bend_speed_right;
	record->bend_speed_left = LF->Bend_speed_left;
	record->threshold = LF->treshold;
	record->map_p = map->p;
	record->map_i = map->i;
	record->map_d = map->d;
	record->map_default_speed = map->DefaultPlaybackSpeed;
	record->checksum = SettingsChecksum((const uint8_t *)record, offsetof(RobotSettingsRecord_t, checksum));
}

static RobotSettingsStatus_t SettingsValidateRecord(const RobotSettingsRecord_t *record)
{
	if(record->magic == 0xFFFFFFFFu)
	{
		return ROBOT_SETTINGS_EMPTY;
	}
	if((record->magic != ROBOT_SETTINGS_MAGIC) ||
	   (record->version != ROBOT_SETTINGS_VERSION) ||
	   (record->size != sizeof(*record)))
	{
		return ROBOT_SETTINGS_INVALID;
	}

	uint32_t checksum = SettingsChecksum((const uint8_t *)record, offsetof(RobotSettingsRecord_t, checksum));
	if(checksum != record->checksum)
	{
		return ROBOT_SETTINGS_INVALID;
	}

	return ROBOT_SETTINGS_OK;
}

static void SettingsApplyRecord(const RobotSettingsRecord_t *record, LineFollower_t *LF, Map_t *map)
{
	LF->Kp = record->lf_kp;
	LF->Kd = record->lf_kd;
	LF->Speed_offset = record->speed_offset;
	LF->Base_speed_R = record->base_speed_r;
	LF->Base_speed_L = record->base_speed_l;
	LF->Max_speed_R = record->max_speed_r;
	LF->Max_speed_L = record->max_speed_l;
	LF->Sharp_bend_speed_right = record->sharp_bend_speed_right;
	LF->Sharp_bend_speed_left = record->sharp_bend_speed_left;
	LF->Bend_speed_right = record->bend_speed_right;
	LF->Bend_speed_left = record->bend_speed_left;
	LF->treshold = record->threshold;
	map->p = record->map_p;
	map->i = record->map_i;
	map->d = record->map_d;
	map->DefaultPlaybackSpeed = record->map_default_speed;
}

RobotSettingsStatus_t RobotSettings_Load(LineFollower_t *LF, Map_t *map)
{
	const RobotSettingsRecord_t *record = (const RobotSettingsRecord_t *)SettingsFlashAddress();
	RobotSettingsStatus_t status = SettingsValidateRecord(record);

	if(status == ROBOT_SETTINGS_OK)
	{
		SettingsApplyRecord(record, LF, map);
	}

	SettingsLastStatus = status;
	return status;
}

RobotSettingsStatus_t RobotSettings_Save(const LineFollower_t *LF, const Map_t *map)
{
	RobotSettingsRecord_t record;
	SettingsFillRecord(&record, LF, map);

	uint32_t address = SettingsFlashAddress();
	uint32_t page_error = 0u;
	FLASH_EraseInitTypeDef erase = {0};
	erase.TypeErase = FLASH_TYPEERASE_PAGES;
	erase.Banks = SettingsFlashBank(address);
	erase.Page = SettingsFlashPage(address);
	erase.NbPages = 1u;

	HAL_StatusTypeDef hal_status = HAL_FLASH_Unlock();
	if(hal_status != HAL_OK)
	{
		SettingsLastStatus = ROBOT_SETTINGS_FLASH_ERROR;
		return SettingsLastStatus;
	}

	hal_status = HAL_FLASHEx_Erase(&erase, &page_error);
	if(hal_status == HAL_OK)
	{
		const uint8_t *raw = (const uint8_t *)&record;
		for(uint32_t offset = 0u; offset < sizeof(record); offset += 8u)
		{
			uint64_t double_word = 0xFFFFFFFFFFFFFFFFull;
			uint32_t remaining = sizeof(record) - offset;
			uint32_t copy_len = (remaining >= 8u) ? 8u : remaining;
			memcpy(&double_word, &raw[offset], copy_len);

			hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + offset, double_word);
			if(hal_status != HAL_OK)
			{
				break;
			}
		}
	}

	(void)HAL_FLASH_Lock();

	if(hal_status == HAL_OK)
	{
		SettingsSaveDirty = 0u;
		SettingsLastStatus = ROBOT_SETTINGS_OK;
	}
	else
	{
		SettingsLastStatus = ROBOT_SETTINGS_FLASH_ERROR;
	}

	return SettingsLastStatus;
}

void RobotSettings_RequestSave(void)
{
	SettingsSaveDirty = 1u;
	SettingsSaveRequestTick = HAL_GetTick();
}

void RobotSettings_Service(const LineFollower_t *LF, const Map_t *map)
{
	if(SettingsSaveDirty == 0u)
	{
		return;
	}
	if(LF->PowerMode == Start)
	{
		return;
	}
	if((HAL_GetTick() - SettingsSaveRequestTick) < ROBOT_SETTINGS_SAVE_DELAY)
	{
		return;
	}

	(void)RobotSettings_Save(LF, map);
}

RobotSettingsStatus_t RobotSettings_LastStatus(void)
{
	return SettingsLastStatus;
}

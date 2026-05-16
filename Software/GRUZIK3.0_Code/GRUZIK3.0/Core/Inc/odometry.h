/*
 * odometry.h
 *
 * Encoder + gyro fused odometry for GRUZIK3.0.
 */

#ifndef INC_ODOMETRY_H_
#define INC_ODOMETRY_H_

#include "main.h"

typedef struct
{
	float x_m;
	float y_m;
	float yaw_rad;
} Pose2D_t;

typedef struct
{
	Pose2D_t pose;
	float gyro_yaw_rad;
	float encoder_yaw_rad;
	float last_delta_yaw_encoder_rad;
	float last_delta_yaw_gyro_rad;
	float last_delta_distance_m;
	float total_distance_m;
	float gyro_weight;
} Odometry_t;

void Odometry_Init(Odometry_t *odom, float gyro_weight);
void Odometry_Reset(Odometry_t *odom);
void Odometry_Update(Odometry_t *odom, float left_delta_m, float right_delta_m, float gyro_z_dps, float dt_s);
float Odometry_NormalizeAngle(float angle_rad);

#endif /* INC_ODOMETRY_H_ */

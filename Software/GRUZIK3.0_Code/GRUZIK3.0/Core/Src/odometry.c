/*
 * odometry.c
 *
 * Encoder + gyro fused odometry for GRUZIK3.0.
 */

#include "odometry.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

float Odometry_NormalizeAngle(float angle_rad)
{
	while(angle_rad > M_PI)
	{
		angle_rad -= 2.0f * M_PI;
	}
	while(angle_rad <= -M_PI)
	{
		angle_rad += 2.0f * M_PI;
	}
	return angle_rad;
}

void Odometry_Init(Odometry_t *odom, float gyro_weight)
{
	memset(odom, 0, sizeof(*odom));
	if(gyro_weight < 0.0f)
	{
		gyro_weight = 0.0f;
	}
	if(gyro_weight > 1.0f)
	{
		gyro_weight = 1.0f;
	}
	odom->gyro_weight = gyro_weight;
}

void Odometry_Reset(Odometry_t *odom)
{
	float gyro_weight = odom->gyro_weight;
	memset(odom, 0, sizeof(*odom));
	odom->gyro_weight = gyro_weight;
}

void Odometry_Update(Odometry_t *odom, float left_delta_m, float right_delta_m, float gyro_z_dps, float dt_s)
{
	float delta_distance = 0.5f * (left_delta_m + right_delta_m);
	float delta_yaw_encoder = ((right_delta_m - left_delta_m) / ROBOT_WHEEL_BASE_M) * ROBOT_ENCODER_YAW_DIRECTION;
	float delta_yaw_gyro = gyro_z_dps * DEG_TO_RAD * dt_s;
	float fused_delta_yaw = (odom->gyro_weight * delta_yaw_gyro) + ((1.0f - odom->gyro_weight) * delta_yaw_encoder);
	float previous_yaw = odom->pose.yaw_rad;
	float integration_yaw = previous_yaw + (0.5f * fused_delta_yaw);

	odom->pose.x_m += delta_distance * cosf(integration_yaw);
	odom->pose.y_m += delta_distance * sinf(integration_yaw);
	odom->pose.yaw_rad = Odometry_NormalizeAngle(previous_yaw + fused_delta_yaw);

	odom->gyro_yaw_rad = Odometry_NormalizeAngle(odom->gyro_yaw_rad + delta_yaw_gyro);
	odom->encoder_yaw_rad = Odometry_NormalizeAngle(odom->encoder_yaw_rad + delta_yaw_encoder);
	odom->last_delta_yaw_encoder_rad = delta_yaw_encoder;
	odom->last_delta_yaw_gyro_rad = delta_yaw_gyro;
	odom->last_delta_distance_m = delta_distance;
	odom->total_distance_m += fabsf(delta_distance);
}

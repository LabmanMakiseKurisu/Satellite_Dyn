/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:11
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 18:28:02
 * @FilePath: /Satellite/src/Componet/Sensors/Gyro.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include "Sensor.hh"

class GyroScope : public Sensor<Eigen::Vector3d, Eigen::Vector3d>
{
public:
	GyroScope(Eigen::Vector3d *_s,
	Eigen::Matrix3d &IM, 
	int64_t timestamp, 
	double Ts, int _id, 
	int _size = 3, std::string Code = "SIM04");
public:
	virtual void StateRenew(int64_t NowTime) override;
	virtual void Init(int64_t timestamp) override;
	virtual void Submit() override;
};


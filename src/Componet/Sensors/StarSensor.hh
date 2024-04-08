/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:11
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 11:29:44
 * @FilePath: /Satellite/src/Componet/Sensors/StarSensor.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include"Quaternions.hh"
#include"Dcm.hh"
#include "Sensor.hh"
class StarSensor : public Sensor<Quat, Quat>
{
public:
	StarSensor(Quat *_s,
			   Eigen::Matrix3d &IM,
			   int64_t timestamp,
			   double Ts, int _id,
			   int _size = 4, std::string Code = "SIM05");
	CDcm InstallMatrix;

public:
	virtual void StateRenew(int64_t NowTime) override;
	virtual void Init(int64_t timestamp) override;
	virtual void Submit() override;
};
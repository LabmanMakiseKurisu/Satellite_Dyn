/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:11
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 20:10:05
 * @FilePath: /Satellite/src/Componet/StarSensor.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include"Quaternions.hh"
#include"Dcm.hh"
#include "Sensor.hh"
class StarSensor : public Sensor<Quat>
{
public:
	StarSensor(Eigen::Matrix3d &IM, int64_t timestamp, double Ts, int _id, int _size=4, std::string Code = "SIM05");
	CDcm InstallMatrix;

public:
	virtual void StateRenew(int64_t NowTime, Quat& Qib) override;
	virtual void Init(int64_t timestamp,Quat& Qib) override;
	virtual void Submit() override;
};
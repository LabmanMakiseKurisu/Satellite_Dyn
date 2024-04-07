/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:11
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 20:10:01
 * @FilePath: /Satellite/src/Componet/SunSensor.hh
 * @Description:
 */
#pragma once
#include "Sensor.hh"

class SunSensor : public Sensor<Eigen::Vector3d>
{
public:
	SunSensor(Eigen::Matrix3d& IM, int64_t timestamp, double Ts, int _id, int _size=3,std::string Code="SIM06");

public:
	virtual void StateRenew(int64_t NowTime, Eigen::Vector3d& Sun_b) override;
	virtual void Init(int64_t timestamp,Eigen::Vector3d& Sun_b) override;
	virtual void Submit() override;
};

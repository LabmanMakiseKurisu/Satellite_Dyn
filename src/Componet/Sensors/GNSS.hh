/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:11
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 11:30:14
 * @FilePath: /Satellite/src/Componet/Sensors/GNSS.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include "Orbit.hh"
#include "Sensor.hh"

class GNSS : public Sensor<RV, RV>
{
public:
	GNSS(RV *_s,
		 Eigen::Matrix3d &IM,
		 int64_t timestamp,
		 double Ts, int _id,
		 int _size = 6, std::string Code = "SIM08");

public:
	virtual void StateRenew(int64_t NowTime) override;
	virtual void Init(int64_t timestamp) override;
	virtual void Submit() override;
};

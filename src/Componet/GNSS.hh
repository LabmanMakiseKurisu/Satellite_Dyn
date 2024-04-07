/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:11
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 20:24:25
 * @FilePath: /Satellite/src/Componet/GNSS.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include "Orbit.hh"
#include "Sensor.hh"

class GNSS : public Sensor<RV>
{
public:
	GNSS(Eigen::Matrix3d &IM, int64_t timestamp, double Ts, int _id, int _size = 6, std::string Code = "SIM08");

public:
	virtual void StateRenew(int64_t NowTime, RV& InlRV) override;
	virtual void Init(int64_t timestamp,RV& InlRV) override;
	virtual void Submit() override;
};



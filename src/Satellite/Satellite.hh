/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:23
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 16:55:07
 * @FilePath: /Satellite/src/Satellite/Satellite.hh
 * @Description: 
 */
#pragma once
#include"Attitude.hh"
#include"Orbit.hh"
#include "Environment.hh"
#include "AttitudeControl.hh"
#include"Componet.hh"


class Satellite
{
public:
	int64_t SatelliteTime;//ms
	COrbit Orbit;//
	CAttitude Attitude;//
	Environment Env;//
	CComponet* pComponet;//
	CAttitudeController AttController;//
public:
	double SampleTime;//
	int SpeedTimes;//
public:
	Satellite();
	Satellite(double Ts, int m_SpeedTimes);
	~Satellite()=default;
	void StateRenew();
	// 2023-12-22 11:28:55
	void data2DB(CInfluxDB& DB, double Period);
	void record(CInfluxDB& DB);
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);
/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:23
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 18:38:33
 * @FilePath: /Satellite/src/Satellite/Satellite.hh
 * @Description: 
 */
#pragma once
#include"Attitude.hh"
#include"Orbit.hh"
#include "Environment.hh"
#include "AttitudeControl.hh"
#include"Com_Schedule.hh"
#include"Subscriber.hh"

class Satellite: public ::ISubscriber
{
public:
	int64_t SatelliteTime;//ms
	COrbit Orbit;//
	CAttitude Attitude;//
	Environment Env;//
	Com_Schedule* pComponet;//
	CAttitudeController AttController;//
public:
	double m_Delta;//
	int m_Rate;//
public:
	Satellite();
	void Init();
	~Satellite()=default;
	void StateRenew();
	virtual void Submit() override;
};

std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat);
/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-06 19:28:11
 * @FilePath: /Satellite/src/Satellite/Satellite.cpp
 * @Description: 
 */
#include "Satellite.h"
#include"CConfig.h"
#include"InfluxDB.h"
#include"sofaDLL.h"
Satellite::Satellite() :Orbit(), Attitude(), AttController()
{
	SampleTime = 0.1;
	SpeedTimes = 1;
	CConfig* pCfg = CConfig::GetInstance();

	//?????????
	SatelliteTime = pCfg->SatelliteTime;

	//??????????
	Orbit.Init(SatelliteTime);

	//??????????
	Attitude.Init(Orbit);

	//???????????
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//??????????
	AttController.workmode = CAttitudeController::EARTHPOINT;

	//?????????
	pComponet = CComponet::GetInstance();
	pComponet->Init(Attitude, Orbit, Env, AttController,SatelliteTime);
}

Satellite::Satellite(double Ts, int m_SpeedTimes):Satellite()
{
	SampleTime = Ts;
	SpeedTimes = m_SpeedTimes;
}


void Satellite::StateRenew()
{
	//????????
	SatelliteTime += (int64_t)(SampleTime * 1e3);

	//??????????
	AttController.TorqueRefRenew(pComponet);

	//?????????????
	Orbit.StateRenew(SampleTime, SatelliteTime);

	//?????????????
	Attitude.StateRenew(SampleTime, Orbit, pComponet);

	//???????????
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//???????????
	pComponet->StateRenew(Attitude, Orbit, Env, AttController, SatelliteTime, SampleTime);

}

void Satellite::data2DB(CInfluxDB& DB, double Period)
{
	// -->Period???????????
	if (DB.IsSend(Period)) {
		DB.ResetStr2();
		this->record(DB);
		Orbit.record(DB);
		Attitude.record(DB);
		Env.record(DB);
		pComponet->record(DB);
		AttController.record(DB);
		DB.sendUdp();
	}
}

void Satellite::record(CInfluxDB& DB)
{
	DB.addKeyValue("SIM001", SampleTime);
	DB.addKeyValue("SIM002", SpeedTimes);
	DB.addKeyValue("SIM003", SatelliteTime);
}


std::ostream& operator<<(std::ostream& _cout, const Satellite& Sat)
{
	YMD ymd = UTCTimeStamp2YMD(Sat.SatelliteTime);
	std::cout << std::fixed;
	//_cout << "SatelliteTime(ms) " << Sat.SatelliteTime << std::endl;
	_cout << "Time " << ymd.year << " " << ymd.month << " " << ymd.day << " " << ymd.hour << ":" << ymd.minute << ":" << ymd.second << std::endl;
	_cout << Sat.Orbit.J2000Inertial << std::endl;
	//_cout << "FIXRV" << Sat.Orbit.ECEFFix << std::endl;
	//_cout << "Omega_b(rad/s) " << Sat.Attitude.Omega_b(0) << " " << Sat.Attitude.Omega_b(1) << " " << Sat.Attitude.Omega_b(2) << std::endl;//??????????????
	//_cout << "Qib " << Sat.Attitude.Qib;
	//_cout << "Qob " << Sat.Attitude.Qob;
	//_cout << "TotalTorque(N.m) " << Sat.Attitude.TotalTorque(0) << " " << Sat.Attitude.TotalTorque(1) << " " << Sat.Attitude.TotalTorque(2) << std::endl;
	//_cout<<"VSunBody"<< Sat.Env.SunVecBody(0)<<" "<< Sat.Env.SunVecBody(1) <<" "<< Sat.Env.SunVecBody(2) << std::endl;
	//_cout << "VSunInl" << Sat.Env.SunVecInl(0) << " " << Sat.Env.SunVecInl(1) << " " << Sat.Env.SunVecInl(2) << std::endl;
	return _cout;
}
/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 14:44:55
 * @FilePath: /Satellite/src/Satellite/Satellite.cc
 * @Description: 
 */
#include "Satellite.hh"
#include"InfluxDB.hh"
#include"sofaDLL.h"
#include"GlobalSetting.hh"
#include"Mediator.hh"
Satellite::Satellite() :Orbit(), Attitude(), AttController()
{
	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	m_Delta = pCfg->Get<double>("/Satellite/Delta");
	m_Rate = pCfg->Get<int>("/Satellite/Rate");

	SatelliteTime = pCfg->Get<int64_t>("/Satellite/SatelliteTime");

}

double *Satellite::Addr(int index)
{
	switch (index)
	{
	case 0:
		return &m_Delta;
		break;
	case 1:
		return &m_Rate;
		break;
	case 2:
		return &SatelliteTime;
		break;
	default:
		return nullptr;
	}
}

void Satellite::Init() {
	Orbit.Init(SatelliteTime);

	Attitude.Init(Orbit);

	Env.Init();

	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	AttController.Init();

	AttController.workmode = CAttitudeController::EARTHPOINT;

	pComponet = Com_Schedule::GetInstance();
	pComponet->Init(Attitude, Orbit, Env, AttController,SatelliteTime);

	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	StartCode = pCfg->Get<std::string>("/Satellite/StartCode");
	fileds = pCfg->Get<int>("/Satellite/fields");

	m_DM = Publisher::GetInstance();
	m_DM->Subscribe(this);

	auto hd = Mediator::GetInstance();
	for (int i = 0; i < fileds; i++)
	{
		std::string Code = GetCode(StartCode, i + 1);
		hd->add(Code, Addr(i));
	}
}

void Satellite::StateRenew()
{
	//
	SatelliteTime += (int64_t)(m_Delta * 1e3);

	//
	AttController.TorqueRefRenew(pComponet);

	//
	Orbit.StateRenew(m_Delta, SatelliteTime);

	//
	Attitude.StateRenew(m_Delta, Orbit, pComponet);

	//
	Env.StateRenew(Attitude, Orbit, SatelliteTime);

	//
	pComponet->StateRenew(SatelliteTime, AttController);

}
void Satellite::Submit()
{
	if (!m_DM)
		return;
	int index = 1;
	m_DM->add<double>(GetCode(StartCode, index), m_Delta);
	index++;
	m_DM->add<double>(GetCode(StartCode, index), m_Rate);
	index++;
	m_DM->add<double>(GetCode(StartCode, index), SatelliteTime);
	index++;
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
#include "AttitudeControl.hh"
#include"Dcm.hh"
#include"Quaternions.hh"
#include"Attitude.hh"
#include "Environment.hh"
#include"InfluxDB.hh"
#include "Com_Schedule.hh"
#include "Gyro.hh"
#include "SunSensor.hh"
#include "StarSensor.hh"
#include "MagSensor.hh"
#include "GNSS.hh"
#include "Flywheel.hh"
#include"Mediator.hh"
#include"GlobalSetting.hh"

using AttCtrl = CAttitudeController;
CAttitudeController::CAttitudeController() :workmode(EARTHPOINT)
{
	TorqueRef << 0, 0, 0;
	Kp << 0.5 * Eigen::Matrix3d::Identity();
	Kd << 3 * Eigen::Matrix3d::Identity();
	MaxTorque = 0.08;
	m_mode = workmode;
}

double *CAttitudeController::Addr(int index)
{
	switch (index)
	{
	case 0:
		return &m_mode;
	case 1:
	case 2:
	case 3:
		return &TorqueRef[index - 1];
	case 4:
	case 5:
	case 6:
		return &Kp(index - 4, 0);
	case 7:
	case 8:
	case 9:
		return &Kd(index - 7, 0);
	}
	return nullptr;
}
void CAttitudeController::Init() {
	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	StartCode = pCfg->Get<std::string>("/AttCtrl/StartCode");
	fileds = pCfg->Get<int>("/AttCtrl/fields");

	m_DM = Publisher::GetInstance();
	m_DM->Subscribe(this);

	auto hd = Mediator::GetInstance();
	for (int i = 0; i < fileds; i++)
	{
		std::string Code = GetCode(StartCode, i + 1);
		hd->add(Code, Addr(i));
	}
}

Eigen::Vector3d CAttitudeController::TorqueRefRenew(Com_Schedule* pCom)
{
	switch ((int)m_mode)
	{
	case RATEDAMP:
	{
		RateDamping(*pCom->Gyros[0]);
	}
	break;
	case SUNPOINT:
	{
		ToSunControl(*pCom->Gyros[0], *pCom->SunSensors[0]);
	}
	break;
	case EARTHPOINT:
	{

		ToEarthControl(*(pCom->Gyros[0]), *(pCom->StarSensors[0]), *pCom->GNSSs[0]);
	}
	break;
	default:
		TorqueRef << 0, 0, 0;
		break;
	}
	return TorqueRef;
}

void CAttitudeController::RateDamping(const GyroScope& _Gyro)
{

	Eigen::Vector3d Tcontrol = -Kp * _Gyro.GetInstallMatrix().inverse() * DEG2RAD * _Gyro.GetData();
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}


void CAttitudeController::ToSunControl(const GyroScope& _Gyro, const SunSensor& _Sun)
{
	Eigen::Vector3d Wbi = _Gyro.GetInstallMatrix().inverse() * DEG2RAD * _Gyro.GetData();


	Eigen::Vector3d Wref(0, 0, 0.1 * DEG2RAD);
	Eigen::Vector3d Rb(0, 0, 1);
	Eigen::Vector3d _SunPos = _Sun.GetInstallMatrix().inverse() * _Sun.GetData();
	//
	Eigen::Vector3d Tcontrol = -Kp * _SunPos.cross(Rb) + Kd * (Eigen::Vector3d::Zero() - Wbi);
	//
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}

//
void CAttitudeController::ToEarthControl(const GyroScope& _Gyro, const StarSensor& _Star, const GNSS& _gnss)
{
	Eigen::Vector3d Wbi = _Gyro.GetInstallMatrix().inverse() * DEG2RAD * _Gyro.GetData();
	Quat Qib = _Star.GetData() * _Star.InstallMatrix.ToQuat();

	//Aio
	CDcm Aio = CAttitude::GetAio(_gnss.GetData());

	Quat Qoi = Aio.ToQuat().QuatInv();
	Quat Qob = Qoi * Qib;
	Eigen::Vector3d ImQob;
	ImQob << Qob.QuatData[1], Qob.QuatData[2], Qob.QuatData[3];

	//
	Eigen::Vector3d Tcontrol = -Kp * ImQob - Kd * Wbi;
	//
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}
void CAttitudeController::Submit()
{
    if (!m_DM)
        return;
    int index = 1;
	m_DM->add<int>(GetCode(StartCode,index), (int)m_mode);
	index++;

	for(int i = 0; i < 3; i++) {
		m_DM->add<double>(GetCode(StartCode,index), TorqueRef[i]);
		index++;
	}
}
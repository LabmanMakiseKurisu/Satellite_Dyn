#include "Com_Schedule.hh"
#include "Attitude.hh"
#include "Orbit.hh"
#include "Environment.hh"
#include "AttitudeControl.hh"
#include "InfluxDB.hh"
#include "GlobalSetting.hh"
#include "Gyro.hh"
#include "SunSensor.hh"
#include "StarSensor.hh"
#include "MagSensor.hh"
#include "GNSS.hh"
#include "Flywheel.hh"
#include "Subscriber.hh"

Com_Schedule::DeleteHelper Com_Schedule::helper;

static Com_Schedule* pComponet = Com_Schedule::GetInstance();


Com_Schedule* Com_Schedule::GetInstance()
{
	if (m_instance == NULL)
		m_instance = new Com_Schedule;
	return m_instance;
}

void Com_Schedule::Init(CAttitude& Att, COrbit& Obt, Environment& Env, CAttitudeController& ACtrl, int64_t timestamp)
{
	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	for (size_t i{ 1 }; i <= GyroNums; i++)
	{
		std::string GytoStr = "/Gyro/InstallMatrix" + std::to_string(i);
		Eigen::Matrix3d IM = pCfg->GetMat(GytoStr);
		Gyros[i - 1] = new GyroScope(&Att.Omega_b, IM, timestamp, pCfg->Get<double>("/Gyro/SamplePeriod"), i);
		Gyros[i - 1]->Init(timestamp);
		//Gyros[i - 1]->m_DM->Subscribe(Gyros[i - 1]);
	}

	for (size_t i{ 1 }; i <= SunSensorNums; i++)
	{
		std::string SunSensorStr = "/SunSensor/InstallMatrix" + std::to_string(i);
		Eigen::Matrix3d IM = pCfg->GetMat(SunSensorStr);
		SunSensors[i - 1] = new SunSensor(&Env.SunVecBody, IM, timestamp, pCfg->Get<double>("/SunSensor/SamplePeriod"), i);
		SunSensors[i - 1]->Init(timestamp);
		//SunSensors[i-1]->m_DM->Subscribe(SunSensors[i-1]);
	}

	for (size_t i{ 1 }; i <= StarSensorNums; i++)
	{
		std::string StarSensorStr = "/StarSensor/InstallMatrix" + std::to_string(i);
		Eigen::Matrix3d IM = pCfg->GetMat(StarSensorStr);
		StarSensors[i - 1] = new StarSensor(&Att.Qib, IM, timestamp, pCfg->Get<double>("/StarSensor/SamplePeriod"), i);
		StarSensors[i - 1]->Init(timestamp);
		//StarSensors[i-1]->m_DM->Subscribe(StarSensors[i-1]);
	}

	for (size_t i{ 1 }; i <= MagSensorNums; i++)
	{
		std::string MagSensorStr = "/MagSensor/InstallMatrix" + std::to_string(i);
		Eigen::Matrix3d IM = pCfg->GetMat(MagSensorStr);
		MagSensors[i - 1] = new MagSensor(&Env.BodyMag, IM, timestamp, pCfg->Get<double>("/MagSensor/SamplePeriod"), i);
		MagSensors[i - 1]->Init(timestamp);
		//MagSensors[i - 1]->m_DM->Subscribe(MagSensors[i - 1]);
	}

	for (size_t i{ 1 }; i <= GnssNums; i++)
	{
		Eigen::Matrix3d IM = Eigen::Matrix3d::Identity();
		GNSSs[i - 1] = new GNSS(&Obt.ECEFFix, IM, timestamp, pCfg->Get<double>("/Gnss/SamplePeriod"), i);
		GNSSs[i - 1]->Init(timestamp);
		//GNSSs[i - 1]->m_DM->Subscribe(GNSSs[i - 1]);
	}

	for (size_t i{ 1 }; i <= FlywheelNums; i++)
	{
		std::string WheelStr = "/Flywheel/InstallVector" + std::to_string(i);
		Eigen::Vector3d Iv = pCfg->GetVec(WheelStr);
		Wheels[i - 1] = new Flywheel(&WheelsTref[i], Iv,
		pCfg->Get<double>("/Flywheel/Inertia"), pCfg->Get<double>("/Flywheel/TimeCof"),
		pCfg->Get<double>("/Flywheel/MaxSpeed"), pCfg->Get<double>("/Flywheel/MaxTref"),
		pCfg->Get<double>("/Flywheel/Kp"), pCfg->Get<double>("/Flywheel/Ki"),
		timestamp, i, pCfg->Get<double>("/Satellite/Delta"));
		Wheels[i-1]->Init(timestamp);
		//Wheels[i-1]->m_DM->Subscribe(GNSSs[i - 1]);
	}
}

void Com_Schedule::StateRenew(int64_t timestamp, CAttitudeController &ACtrl)
{
	for (size_t i = 0; i < GyroNums; i++)
	{
		Gyros[i]->StateRenew(timestamp);
	}
	for (size_t i = 0; i < SunSensorNums; i++)
	{
		SunSensors[i]->StateRenew(timestamp);
	}
	for (size_t i = 0; i < StarSensorNums; i++)
	{
		StarSensors[i]->StateRenew(timestamp);
	}
	for (size_t i = 0; i < MagSensorNums; i++)
	{
		MagSensors[i]->StateRenew(timestamp);
	}
	for (size_t i = 0; i < GnssNums; i++)
	{
		GNSSs[i]->StateRenew(timestamp);
	}
	WheelsTrefCal(ACtrl.TorqueRef);
	for (size_t i = 0; i < FlywheelNums; i++)
	{
		Wheels[i]->StateRenew();
	}
}

Com_Schedule::Com_Schedule()
{
	GlobalSettings *pCfg = GlobalSettings::GetInstance();
	GyroNums = pCfg->Get<size_t>("/Gyro/Nums");
	FlywheelNums = pCfg->Get<size_t>("/Flywheel/Nums");
	MagSensorNums = pCfg->Get<size_t>("/MagSensor/Nums");
	SunSensorNums = pCfg->Get<size_t>("/SunSensor/Nums");
	StarSensorNums = pCfg->Get<size_t>("/StarSensor/Nums");
	GnssNums = pCfg->Get<size_t>("/Gnss/Nums");

	Gyros.resize(GyroNums);
	SunSensors.resize(SunSensorNums);
	StarSensors.resize(StarSensorNums);
	MagSensors.resize(MagSensorNums);
	GNSSs.resize(GnssNums);
	Wheels.resize(FlywheelNums);
	WheelsTref.resize(FlywheelNums);
}

Com_Schedule::~Com_Schedule()
{
	for (size_t i = 0; i < GyroNums; i++)
	{
		delete Gyros[i];
	}
	for (size_t i = 0; i < SunSensorNums; i++)
	{
		delete SunSensors[i];
	}
	for (size_t i = 0; i < StarSensorNums; i++)
	{
		delete StarSensors[i];
	}
	for (size_t i = 0; i < MagSensorNums; i++)
	{
		delete MagSensors[i];
	}
	for (size_t i = 0; i < FlywheelNums; i++)
	{
		delete Wheels[i];
	}
	for (size_t i = 0; i < GnssNums; i++)
	{
		delete GNSSs[i];
	}
}

void Com_Schedule::WheelsTrefCal(Eigen::Vector3d &TrefBody)
{
	Eigen::MatrixXd InstallMatrix(3, FlywheelNums);
	for (size_t i{0}; i < FlywheelNums; i++)
	{
		InstallMatrix.col(i) = Wheels[i]->GetInsVet();
	}
	Eigen::MatrixXd tmp1(FlywheelNums, FlywheelNums);
	tmp1 = InstallMatrix * InstallMatrix.transpose();

	Eigen::MatrixXd tmp2(FlywheelNums, 3);
	tmp2 = InstallMatrix.transpose() * tmp1.inverse();

	Eigen::VectorXd Trefs(FlywheelNums);

	Trefs = -tmp2 * TrefBody;

	for (int i = 0; i < FlywheelNums; i++)
		WheelsTref[i] = Trefs[i];
}

void Com_Schedule::ReleaseInstance()
{
	Com_Schedule* tmp = m_instance;
	m_instance = NULL;
	delete tmp;
}


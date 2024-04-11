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
#include"APIHandler.hh"

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
		Gyros[i - 1] = new GyroScope(&Att.Omega_b, IM,
									 timestamp, pCfg->Get<double>("/Gyro/SamplePeriod"),
									 i, pCfg->Get<double>("/Gyro/fields"), pCfg->Get<std::string>("/Gyro/StartCode"));
		Gyros[i - 1]->Init(timestamp);
	}

	for (size_t i{ 1 }; i <= SunSensorNums; i++)
	{
		std::string SunSensorStr = "/SunSensor/InstallMatrix" + std::to_string(i);
		Eigen::Matrix3d IM = pCfg->GetMat(SunSensorStr);
		SunSensors[i - 1] = new SunSensor(&Env.SunVecBody, IM, 
		timestamp, pCfg->Get<double>("/SunSensor/SamplePeriod"), 
		i, pCfg->Get<double>("/SunSensor/fields"), pCfg->Get<std::string>("/SunSensor/StartCode"));
		SunSensors[i - 1]->Init(timestamp);
	}

	for (size_t i{ 1 }; i <= StarSensorNums; i++)
	{
		std::string StarSensorStr = "/StarSensor/InstallMatrix" + std::to_string(i);
		Eigen::Matrix3d IM = pCfg->GetMat(StarSensorStr);
		StarSensors[i - 1] = new StarSensor(&Att.Qib, IM, 
		timestamp, pCfg->Get<double>("/StarSensor/SamplePeriod"), 
		i, pCfg->Get<double>("/StarSensor/fields"), pCfg->Get<std::string>("/StarSensor/StartCode"));
		StarSensors[i - 1]->Init(timestamp);
	}

	for (size_t i{ 1 }; i <= MagSensorNums; i++)
	{
		std::string MagSensorStr = "/MagSensor/InstallMatrix" + std::to_string(i);
		Eigen::Matrix3d IM = pCfg->GetMat(MagSensorStr);
		MagSensors[i - 1] = new MagSensor(&Env.BodyMag, IM, 
		timestamp, pCfg->Get<double>("/MagSensor/SamplePeriod"), 
		i, pCfg->Get<double>("/MagSensor/fields"), pCfg->Get<std::string>("/MagSensor/StartCode"));
		MagSensors[i - 1]->Init(timestamp);
	}

	for (size_t i{ 1 }; i <= GnssNums; i++)
	{
		Eigen::Matrix3d IM = Eigen::Matrix3d::Identity();
		GNSSs[i - 1] = new GNSS(&Obt.ECEFFix, IM, 
		timestamp, pCfg->Get<double>("/Gnss/SamplePeriod"), 
		i, pCfg->Get<double>("/Gnss/fields"), pCfg->Get<std::string>("/Gnss/StartCode"));
		GNSSs[i - 1]->Init(timestamp);
	}

	for (size_t i{ 1 }; i <= FlywheelNums; i++)
	{
		std::string WheelStr = "/Flywheel/InstallVector" + std::to_string(i);
		Eigen::Vector3d Iv = pCfg->GetVec(WheelStr);
		Wheels[i - 1] = new Flywheel(&WheelsTref[i], Iv,
		pCfg->Get<double>("/Flywheel/Inertia"), pCfg->Get<double>("/Flywheel/TimeCof"),
		pCfg->Get<double>("/Flywheel/MaxSpeed"), pCfg->Get<double>("/Flywheel/MaxTref"),
		pCfg->Get<double>("/Flywheel/Kp"), pCfg->Get<double>("/Flywheel/Ki"),
		timestamp, i, pCfg->Get<double>("/Satellite/Delta"),
		pCfg->Get<double>("/Flywheel/fields"), pCfg->Get<std::string>("/Flywheel/StartCode"));
		Wheels[i-1]->Init(timestamp);
	}

	m_DM = Publisher::GetInstance(); 
	m_DM->Subscribe(this);

}

void Com_Schedule::StateRenew(int64_t timestamp, CAttitudeController &ACtrl)
{
    auto RenewSensors = [timestamp](auto&& sensorVectorPtr) {
        for (auto& sensor : *sensorVectorPtr) {
            sensor->StateRenew(timestamp);
        }
    };
    // 遍历Coms向量，对每个存储的向量指针应用RenewSensors操作
	WheelsTrefCal(ACtrl.TorqueRef);
    for (auto& sensorVariant : Coms) {
        std::visit(RenewSensors, sensorVariant);
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

	Coms.push_back(&Gyros);
	Coms.push_back(&StarSensors);
	Coms.push_back(&SunSensors);
	Coms.push_back(&MagSensors);
	Coms.push_back(&GNSSs);
	Coms.push_back(&Wheels);
}

Com_Schedule::~Com_Schedule()
{
	// Lambda表达式，负责删除传感器向量中的所有对象
    auto deleteSensors = [](auto&& sensorVectorPtr) {
        for (auto& sensor : *sensorVectorPtr) {
            delete sensor;
        }
        sensorVectorPtr->clear(); // 清除向量中的指针，这一步是可选的，但是可以保持清洁
    };

    // 遍历Coms向量，对每个存储的向量指针应用deleteSensors操作
    for (auto& sensorVariant : Coms) {
        std::visit(deleteSensors, sensorVariant);
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

void Com_Schedule::Submit() {
	//SIM0400~SIM0900
	std::string Code = "SIM0400";
	// Lambda表达式，负责删除传感器向量中的所有对象
	auto AddSensors = [&Code,this](auto &&sensorVectorPtr)
	{
		m_DM->add<int>(Code, (*sensorVectorPtr).size());
	};

	// 遍历Coms向量，对每个存储的向量指针应用deleteSensors操作
    for (auto& sensorVariant : Coms) {
        std::visit(AddSensors, sensorVariant);
		Code[4]++;
	}

}
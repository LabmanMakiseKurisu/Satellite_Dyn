/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 20:14:23
 * @FilePath: /Satellite/src/Componet/StarSensor.cc
 * @Description: 
 */
#include"StarSensor.hh"
#include"InfluxDB.hh"
StarSensor::StarSensor(Eigen::Matrix3d &IM, int64_t timestamp, double Ts, int _id, int _size, std::string Code)
	: Sensor<Quat>(
		  IM,
		  timestamp,
		  Ts,
		  _id,
		  _size,
		  Code)
{
	InstallMatrix.DcmData = IM;
	Data = Quat(1, 0, 0, 0);
}

void StarSensor::StateRenew(int64_t NowTime, Quat& Qib)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		
		Quat installq;
		installq = InstallMatrix.ToQuat();
		Data = Qib * installq;
		
		LastRenewTime = NowTime;
	}
}

void StarSensor::Init(int64_t timestamp,Quat& Qib)
{
	Quat installq;
	installq = InstallMatrix.ToQuat();
	Data = Qib * installq;
	LastRenewTime = timestamp;
}

void StarSensor::Submit() {
	if(!m_DM)
		return;
	for(int i=1;i<=Fields;i++) {
		int diff = i + (id-1)*Fields;
		std::string Code = StartCode + std::to_string(diff);
		m_DM->add<double>(Code,Data.QuatData[i-1]);
	}
}
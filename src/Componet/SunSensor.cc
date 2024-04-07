/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 20:03:40
 * @FilePath: /Satellite/src/Componet/SunSensor.cc
 * @Description: 
 */
#include"SunSensor.hh"
#include"InfluxDB.hh"
SunSensor::SunSensor(Eigen::Matrix3d &IM, int64_t timestamp, double Ts, int _id, int _size, std::string Code)
	: Sensor<Eigen::Vector3d>(
		  IM,
		  timestamp,
		  Ts,
		  _id,
		  _size,
		  Code)
{
	Data << 0, 0, 0;
}

void SunSensor::StateRenew(int64_t NowTime, Eigen::Vector3d& Sun_b)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		Data = InstallMatrix * Sun_b;
		LastRenewTime = NowTime;
	}
}

void SunSensor::Init(int64_t timestamp,Eigen::Vector3d& Sun_b)
{
	LastRenewTime = timestamp;
	Data = InstallMatrix*Sun_b;
}

void SunSensor::Submit() {
	if(!m_DM)
		return;
	for(int i=1;i<=Fields;i++) {
		int diff = i + (id-1)*Fields;
		std::string Code = StartCode + std::to_string(diff);
		m_DM->add<double>(Code,Data[i-1]);
	}
}
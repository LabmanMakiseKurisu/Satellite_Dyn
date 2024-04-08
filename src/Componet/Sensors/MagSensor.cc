/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 18:03:37
 * @FilePath: /Satellite/src/Componet/Sensors/MagSensor.cc
 * @Description: 
 */
#include"MagSensor.hh"
#include"InfluxDB.hh"
MagSensor::MagSensor(Eigen::Vector3d *_s,
					 Eigen::Matrix3d &IM,
					 int64_t timestamp,
					 double Ts, int _id,
					 int _size, std::string Code)
	: Sensor<Eigen::Vector3d, Eigen::Vector3d>(
		  _s,
		  IM,
		  timestamp,
		  Ts,
		  _id,
		  _size,
		  Code)
{
	Data << 0, 0, 0;
	m_DM->Subscribe(this);
}

void MagSensor::StateRenew(int64_t NowTime)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		Data = T2GAUSS(InstallMatrix * (*source));
		LastRenewTime = NowTime;
	}
}

void MagSensor::Init(int64_t timestamp)
{
	LastRenewTime = timestamp;
	Data = T2GAUSS(InstallMatrix * (*source));
}

void MagSensor::Submit() {
	if(!m_DM)
		return;
	for(int i=1;i<=Fields;i++) {
		int diff = i + (id-1)*Fields;
		std::string Code = StartCode;
		if(diff<10)
			Code.append("0");
		Code.append(std::to_string(diff));
		m_DM->add<double>(Code,Data[i-1]);
	}
}
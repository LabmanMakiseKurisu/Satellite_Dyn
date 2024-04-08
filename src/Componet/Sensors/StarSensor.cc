/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 16:16:33
 * @FilePath: /Satellite/src/Componet/Sensors/StarSensor.cc
 * @Description: 
 */
#include"StarSensor.hh"
#include"InfluxDB.hh"
StarSensor::StarSensor(Quat *_s,
					   Eigen::Matrix3d &IM,
					   int64_t timestamp, double Ts,
					   int _id, int _size, std::string Code)
	: Sensor<Quat, Quat>(
		  _s,
		  IM,
		  timestamp,
		  Ts,
		  _id,
		  _size,
		  Code)
{
	InstallMatrix.DcmData = IM;
	Data = Quat(1, 0, 0, 0);
	m_DM->Subscribe(this);
}

void StarSensor::StateRenew(int64_t NowTime)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		
		Quat installq;
		installq = InstallMatrix.ToQuat();
		Data = (*source) * installq;
		
		LastRenewTime = NowTime;
	}
}

void StarSensor::Init(int64_t timestamp)
{
	Quat installq;
	installq = InstallMatrix.ToQuat();
	Data = (*source) * installq;
	LastRenewTime = timestamp;
}

void StarSensor::Submit() {
	if (!m_DM)
		return;
	for (int i = 1; i <= Fields; i++)
	{
		int diff = i + (id - 1) * Fields;
		std::string Code = StartCode;
		if (diff < 10)
			Code.append("0");
		Code.append(std::to_string(diff));
		m_DM->add<double>(Code, Data.QuatData[i - 1]);
	}
}
/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-11 19:51:30
 * @FilePath: /Satellite/src/Componet/Sensors/Gyro.cc
 * @Description: 
 */
#include "Gyro.hh"
#include"InfluxDB.hh"
#include"APIHandler.hh"
GyroScope::GyroScope(Eigen::Vector3d *_s,
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

void GyroScope::StateRenew(int64_t NowTime)
{
	if (NowTime - LastRenewTime >= SamplePeriod*1e3)
	{
		Data = InstallMatrix * (*source);
		Data = DEG(Data);
		LastRenewTime = NowTime;
	}
}
void GyroScope::Init(int64_t timestamp)
{
	LastRenewTime = timestamp;
	Data = InstallMatrix * (*source);
	Data = DEG(Data);

	auto hd = Handler::GetInstance();
	for (int i = 1; i <= Fields; i++)
	{
		int diff = i + (id - 1) * Fields;
		std::string Code = GetCode(StartCode, diff);
		hd->add(Code, &Data[i - 1]);
	}
}
void GyroScope::Submit() {
	if(!m_DM)
		return;
	for(int i=1;i<=Fields;i++) {
		int diff = i + (id-1)*Fields;
		std::string Code = GetCode(StartCode, diff);
		m_DM->add<double>(Code,Data[i-1]);
	}
}
/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 16:17:08
 * @FilePath: /Satellite/src/Componet/Sensors/GNSS.cc
 * @Description: 
 */
#include "GNSS.hh"
#include"InfluxDB.hh"
GNSS::GNSS(RV *_s,
		   Eigen::Matrix3d &IM,
		   int64_t timestamp,
		   double Ts, int _id,
		   int _size, std::string Code)
	: Sensor<RV, RV>(
		  _s,
		  IM,
		  timestamp,
		  Ts,
		  _id,
		  _size,
		  Code)
{
	Data = RV();
	m_DM->Subscribe(this);
}

void GNSS::StateRenew(int64_t NowTime)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		Data = (*source);
		LastRenewTime = NowTime;
	}
}

void GNSS::Init(int64_t timestamp)
{
	Data = (*source);;
	LastRenewTime = timestamp;
}

void GNSS::Submit() {
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
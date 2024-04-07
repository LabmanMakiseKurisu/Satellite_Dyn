#include "GNSS.hh"
#include"InfluxDB.hh"
GNSS::GNSS(Eigen::Matrix3d &IM, int64_t timestamp, double Ts, int _id, int _size, std::string Code)
	: Sensor<RV>(
		  IM,
		  timestamp,
		  Ts,
		  _id,
		  _size,
		  Code)
{
	Data=RV();
}


void GNSS::StateRenew(int64_t NowTime, RV& InlRV)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		Data = InlRV;
		LastRenewTime = NowTime;
	}
}

void GNSS::Init(int64_t timestamp,RV& InlRV)
{
	Data = InlRV;
	LastRenewTime = timestamp;
}

void GNSS::Submit() {
	if(!m_DM)
		return;
	for(int i=1;i<=Fields;i++) {
		int diff = i + (id-1)*Fields;
		std::string Code = StartCode + std::to_string(diff);
		m_DM->add<double>(Code,Data[i-1]);
	}
}
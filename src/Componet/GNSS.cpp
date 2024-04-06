#include "GNSS.h"

GNSS::GNSS():Data()
{
	LastRenewTime = 0;
	SamplePeriod = 0.25;
}

void GNSS::StateRenew(int64_t NowTime, RV& InlRV)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		Data = InlRV;
		LastRenewTime = NowTime;
	}
}

void GNSS::Init(RV& InlRV, int64_t timestamp)
{
	Data = InlRV;
	LastRenewTime = timestamp;
}



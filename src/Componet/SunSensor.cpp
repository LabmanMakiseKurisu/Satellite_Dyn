#include"SunSensor.h"
SunSensor::SunSensor()
{
	InstallMatrix << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	Data << 0, 0, 0;
	LastRenewTime = 0;
	SamplePeriod = 1;
}

void SunSensor::StateRenew(int64_t NowTime, Eigen::Vector3d Sun_b)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		Data = InstallMatrix * Sun_b;
		LastRenewTime = NowTime;
	}
}

void SunSensor::Init(Eigen::Vector3d Sun_b, int64_t timestamp)
{
	LastRenewTime = timestamp;
	Data = InstallMatrix*Sun_b;
}

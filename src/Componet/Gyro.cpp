#include "Gyro.h"

GyroScope::GyroScope()
{
	InstallMatrix << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;
	Data << 0, 0, 0;
	LastRenewTime = 0;
	SamplePeriod = 1;
}

void GyroScope::StateRenew(int64_t NowTime, Eigen::Vector3d Omega_b)
{
	if (NowTime - LastRenewTime >= SamplePeriod*1e3)
	{
		Data = InstallMatrix * Omega_b;
		Data = DEG(Data);
		LastRenewTime = NowTime;
	}
}

void GyroScope::Init(Eigen::Vector3d& Omega_b, int64_t timestamp)
{
	LastRenewTime = timestamp;
	Data = InstallMatrix * Omega_b;
	Data = DEG(Data);
}

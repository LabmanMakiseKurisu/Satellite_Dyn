#include"StarSensor.h"

StarSensor::StarSensor() :InstallMatrix(1, 0, 0, 0, 1, 0, 0, 0, 1)
{
	Data= Quat( 1, 0, 0, 0 );
	LastRenewTime = 0;
	SamplePeriod = 1;
}

StarSensor::StarSensor(const StarSensor& m)
{
	InstallMatrix = m.InstallMatrix;
	Data = m.Data;
	LastRenewTime = m.LastRenewTime;
	SamplePeriod = m.SamplePeriod;
}

StarSensor& StarSensor::operator=(const StarSensor& m)
{
	if (this != &m)
	{
		InstallMatrix = m.InstallMatrix;
		Data = m.Data;
		LastRenewTime = m.LastRenewTime;
		SamplePeriod = m.SamplePeriod;
	}
	return *this;
}

void StarSensor::StateRenew(int64_t NowTime, Quat Qib)
{
	if (NowTime - LastRenewTime >= SamplePeriod * 1e3)
	{
		
		Quat installq;
		installq = InstallMatrix.ToQuat();
		Data = Qib * installq;
		
		LastRenewTime = NowTime;
	}
}

void StarSensor::Init(Quat Qib, int64_t timestamp)
{
	Quat installq;
	installq = InstallMatrix.ToQuat();
	Data = Qib * installq;
	LastRenewTime = timestamp;
}

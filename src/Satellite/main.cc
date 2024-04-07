/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:36
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 18:38:52
 * @FilePath: /Satellite/src/Satellite/main.cc
 * @Description: 
 */
#include "SimTime.hh"
#include "Satellite.hh"
#include"InfluxDB.hh"
int main(int argc, char* argv[])
{
	double SampleTime{ 0.1 };
	int SpeedTimes = 1;
	Satellite Amadeus(SampleTime, SpeedTimes);
	CSimTime* pSimTime = CSimTime::GetInstance();
	pSimTime->Init(SampleTime, SpeedTimes);
	while (1)
	{
		if (pSimTime->check())
		{
			Amadeus.StateRenew();
			std::cout << Amadeus;
		}
		DataManager::GetInstance()->write();
		SleepMs(1);
	}	
	return 0;
}

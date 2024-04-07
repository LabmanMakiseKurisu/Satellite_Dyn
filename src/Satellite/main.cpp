/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:36
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 12:15:10
 * @FilePath: /Satellite/src/Satellite/main.cpp
 * @Description: 
 */
#include "SimTime.h"
#include "Satellite.h"
#include"InfluxDB.h"
#include "IniConfig.h"
int main(int argc, char* argv[])
{
	CInfluxDB DB;
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
		Amadeus.data2DB(DB, 1);
		SleepMs(1);
	}	
	return 0;
}


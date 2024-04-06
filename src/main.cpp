/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:36
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-06 13:26:22
 * @FilePath: /Satellite/src/main.cpp
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
	pSimTime->InitSimSpeedManage(SampleTime, SpeedTimes);
	while (1)
	{
		pSimTime->WaitForSimCountMute();
		if (pSimTime->SimCountJudge())
		{
			Amadeus.StateRenew();
			std::cout << Amadeus;
		}
		pSimTime->ReleaseSimCountMute();
		Amadeus.data2DB(DB, 1);
	}	
	return 0;
}
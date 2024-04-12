/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:36
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 13:08:53
 * @FilePath: /Satellite/src/Satellite/main.cc
 * @Description: 
 */
#include "SimTime.hh"
#include "Satellite.hh"
#include"InfluxDB.hh"
#include"APIServer.hh"
int main(int argc, char* argv[])
{

	Satellite Amadeus;
	Amadeus.Init();
	CSimTime* pSimTime = CSimTime::GetInstance();
	pSimTime->Init(&Amadeus.m_Delta, &Amadeus.m_Rate);
	APIServer Server;
	Server.run();
	while (1)
	{
		if (pSimTime->check())
		{
			Amadeus.StateRenew();
			//std::cout << Amadeus;
		}
		Publisher::GetInstance()->write();
		SleepMs(1);
	}	
	return 0;
}

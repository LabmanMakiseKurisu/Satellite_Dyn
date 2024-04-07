/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 12:22:20
 * @FilePath: /Satellite/src/General/SimTime.cpp
 * @Description: 
 */
#include "SimTime.h"
#include<chrono>
#include <thread>

CSimTime::DeleteHelper CSimTime::helper;
CSimTime* m = CSimTime::GetInstance();

int64_t GetTimeStampMs()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void SleepMs(int64_t milliseconds) {
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void CSimTime::Core() {
	CSimTime* instance = CSimTime::GetInstance();
	int64_t NexTime = GetTimeStampMs();
	int64_t WaitTime = 0;
	while (!m_stop)
	{
		int64_t NowTime = GetTimeStampMs();
		WaitTime = NexTime - NowTime;
		if (WaitTime > 0)
			SleepMs(static_cast<uint32_t>(WaitTime));
		instance->m_SimCount.fetch_add(instance->m_Rate);
		NexTime += static_cast<int64_t>(instance->m_Delta * 1e3);
	}
}

CSimTime* CSimTime::GetInstance()
{
		if (m_instance == NULL)
			m_instance = new CSimTime;
		return m_instance;	
}



void CSimTime::Init(double Delta, int Rate)
{
	CSimTime* instance = CSimTime::GetInstance();
	instance->m_Delta = Delta;
	instance->m_Rate = Rate;
	m_thread = std::thread(&CSimTime::Core,this);
}

//不会有其他线程减，所以只要check>0就行
bool CSimTime::check()
{
	int times = m_SimCount.load();
	if (times > 0)
	{
		m_SimCount--;
		return true;
	}
	else
		return false;
}

CSimTime::CSimTime():m_Delta(0.5), m_Rate(1),m_stop(false)
{
	m_SimCount.store(0);
}

void CSimTime::ReleaseInstance()
{
	if (CSimTime::m_instance != nullptr) {
		delete CSimTime::m_instance;
		CSimTime::m_instance = nullptr;
	}
}

CSimTime::~CSimTime()
{
	m_stop = true;
	m_thread.join();
}

/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:52:34
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-02-26 10:15:26
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
#ifdef _WIN32
unsigned __stdcall SimCountManage(void* arg) {
#else
void* CSimTime::SimCountManage(void* arg) {
#endif
	CSimTime* instance = CSimTime::GetInstance();
	int64_t NexTime = GetTimeStampMs();
	int64_t WaitTime = 0;
	while (1)
	{
		int64_t NowTime = GetTimeStampMs();
		WaitTime = NexTime - NowTime;
		if (WaitTime > 0)
			SleepMs(static_cast<uint32_t>(WaitTime));
		instance->WaitForSimCountMute();
		instance->SimCount += instance->SpeedTimes;
		instance->ReleaseSimCountMute();
		NexTime += static_cast<int64_t>(instance->SampleTime * 1e3);
	}

#ifdef _WIN32
    return 0;
#else
    return nullptr;
#endif
}

CSimTime* CSimTime::GetInstance()
{
		if (m_instance == NULL)
			m_instance = new CSimTime;
		return m_instance;	
}

int CSimTime::WaitForSimCountMute()
{
#ifdef _WIN32
    return WaitForSingleObject(hSimCountMute, INFINITE);
#else
    return pthread_mutex_lock(&hSimCountMute);
#endif
}

int CSimTime::ReleaseSimCountMute()
{
#ifdef _WIN32
    return ReleaseMutex(hSimCountMute);
#else
    return pthread_mutex_unlock(&hSimCountMute);
#endif
}

void CSimTime::InitSimSpeedManage(double SampleTime, int SpeedTimes)
{
	CSimTime* instance = CSimTime::GetInstance();
	instance->SampleTime = SampleTime;
	instance->SpeedTimes = SpeedTimes;
	#ifdef _WIN32
    _beginthreadex(NULL, 0, SimCountManage, NULL, 0, NULL);
#else
    pthread_t thread;
    pthread_create(&thread, nullptr, SimCountManage, this);
    pthread_detach(thread); // Ensure thread is detached
#endif
}

bool CSimTime::SimCountJudge()
{
	if (SimCount > 0)
	{
		SimCount--;
		return true;
	}
	else
		return false;
}

CSimTime::CSimTime():SimCount(0), SampleTime(0.5), SpeedTimes(1)
{
	#ifdef _WIN32
    hSimCountMute = CreateMutex(NULL, FALSE, NULL);
	if (hSimCountMute == NULL)
	{
		std::cout << "����������ʧ�ܣ������˳�\n";
		exit(0);
	}
#else
    pthread_mutex_init(&hSimCountMute, nullptr);
#endif
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
	#ifdef _WIN32
    CloseHandle(hSimCountMute);
#else
    pthread_mutex_destroy(&hSimCountMute);
#endif
}

/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:53:10
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-02-26 10:48:14
 * @FilePath: /Satellite/include/General/SimTime.h
 * @Description: 
 */
#pragma once
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#include<process.h>
#else
#include <pthread.h>
#endif

class CSimTime
{
public:
	static CSimTime* GetInstance();
	int WaitForSimCountMute();
	int ReleaseSimCountMute();
	void InitSimSpeedManage(double SampleTime, int SpeedTimes);
	bool SimCountJudge();
private:
	double SampleTime;//����ʱ�䣬��λ��s;
	int SpeedTimes;//���ٱ���
	int SimCount;
	#ifdef _WIN32
    HANDLE hSimCountMute;
#else
    pthread_mutex_t hSimCountMute;
#endif

	static inline CSimTime* m_instance{ NULL };
	CSimTime();
	~CSimTime();
	CSimTime(const CSimTime& _CSimTime) = delete;
	CSimTime& operator=(const CSimTime& _CSimTime) = delete;

	static void ReleaseInstance();

#ifdef _WIN32
    friend unsigned __stdcall SimCountManage(void* arg);
#else
    static void* SimCountManage(void* arg);
#endif
	class DeleteHelper
	{
	public:
		DeleteHelper() = default;
		~DeleteHelper()
		{
			ReleaseInstance();
		}
	};
	static DeleteHelper helper;
};

//
//brief:��ȡ��ǰϵͳʱ�������λms
//
int64_t GetTimeStampMs();

void SleepMs(int64_t milliseconds);
/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:53:10
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 14:45:24
 * @FilePath: /Satellite/src/General/SimTime.hh
 * @Description: 
 */
#pragma once
#include <iostream>
#include<atomic>
#include<thread>
class CSimTime
{
public:
	static CSimTime* GetInstance();
	void Init(double* Delta, double* Rate);
	bool check();

private:
	double* m_Delta;//积分时长
	double* m_Rate;//加速倍率
	std::atomic<int> m_SimCount;
	std::thread m_thread;
	bool m_stop;
	static inline CSimTime* m_instance{ NULL };
	CSimTime();
	~CSimTime();
	CSimTime(const CSimTime& _CSimTime) = delete;
	CSimTime& operator=(const CSimTime& _CSimTime) = delete;
    void Core();

	static void ReleaseInstance();
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

// 获取时间戳ms
int64_t GetTimeStampMs();

// 休眠ms
void SleepMs(int64_t milliseconds);
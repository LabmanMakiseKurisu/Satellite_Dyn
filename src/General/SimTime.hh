/*
 * @Author: Amadeus
 * @Date: 2024-02-26 08:53:10
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 12:19:52
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
	void Init(double Delta, int Rate);
	bool check();
	void SetRate(int Rate) { m_Rate = Rate; }
	void SetDelta(double Delta) { m_Delta = Delta; }

private:
	double m_Delta;//积分时长
	int m_Rate;//加速倍率
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
#pragma once
#include"BaseMath.h"
#include"Quaternions.h"
#include"Dcm.h"
class StarSensor
{
public:
	CDcm InstallMatrix;//��װ����,�ӱ���ϵ������ϵ
	Quat Data;//������ֵ
	int64_t LastRenewTime;//����ʱ�䣬utcʱ�����λms
	double SamplePeriod;//���ݲ������ڣ���λΪs
public:
	StarSensor();
	~StarSensor() {}
	StarSensor(const StarSensor& m);
	StarSensor& operator=(const StarSensor& m);
public:
	void StateRenew(int64_t NowTime, Quat Qib);
	void Init(Quat Qib, int64_t timestamp);

};


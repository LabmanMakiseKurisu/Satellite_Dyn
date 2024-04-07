#pragma once
#include"BaseMath.h"
class SunSensor
{
public:
	Eigen::Matrix3d InstallMatrix;//��װ����,�ӱ���ϵ������ϵ
	Eigen::Vector3d Data;//̫����������̫��ʸ����ֵ
	int64_t LastRenewTime;//����ʱ�䣬utcʱ�����λms
	double SamplePeriod;//̫���������ڣ���λΪs
public:
	SunSensor();

public:
	void StateRenew(int64_t NowTime, Eigen::Vector3d Sun_b);
	void Init(Eigen::Vector3d Sun_b, int64_t timestamp);
};

#pragma once
#include"BaseMath.h"

class MagSensor
{
public:
	Eigen::Matrix3d InstallMatrix;//��װ����,�ӱ���ϵ������ϵ
	Eigen::Vector3d Data;//��ǿ����ֵ����λGauss
	int64_t LastRenewTime;//����ʱ�䣬utcʱ�����λms
	double SamplePeriod;//�������ڣ���λΪs
public:
	MagSensor();
public:
	void StateRenew(int64_t NowTime, Eigen::Vector3d B_b);
	void Init(Eigen::Vector3d& B_b, int64_t timestamp);
};
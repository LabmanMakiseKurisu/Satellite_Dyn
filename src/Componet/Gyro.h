#pragma once
#include"BaseMath.h"
#include"AllHead.h"


class GyroScope
{
public:
	Eigen::Matrix3d InstallMatrix;//��װ����,�ӱ���ϵ������ϵ
	Eigen::Vector3d Data;//����ͷ��ֵ����λdeg/s
	int64_t LastRenewTime;//�ϴθ���ʱ��,utcʱ���ms
	double SamplePeriod;//���ݲ������ڣ���λΪs
public:
	GyroScope();

public:
	void StateRenew(int64_t NowTime, Eigen::Vector3d Omega_b);
	void Init(Eigen::Vector3d& Omega_b, int64_t timestamp);
};
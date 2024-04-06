#pragma once
#include"BaseMath.h"
class Flywheel
{
public:
	Eigen::Vector3d InstallVet;//��װ�������ӱ���ϵ����װϵ
	double Speed;//����ת����ֵ����λrad/s
	double Torque;//ʵ�����أ���λNm
	double Momentum;//�Ƕ��� ��λNms
	int64_t LastRenewTime;//�ϴθ���ʱ��,utcʱ���ms
	double MaxTref;//���ο�����Nm
	double MaxSpeed;//���ת��
	double SpeedRef;//�ο�ת��rad/s
	double TorqueRef;//�ο����ص�λNm
	//
	double Kp;//���ֿ�����Kp
	double Ki;//���ֿ�����Ki
	double tau;//����ģ��ʱ�䳣��
	double J;//����ת������ kgm2
public:
	Flywheel();
	Flywheel(Eigen::Vector3d& InsVet);
public:
	void StateRenew(int64_t NowTime, double SampleTime, double Tref = 0);
	void Init(double speed, int64_t timestamp);
};


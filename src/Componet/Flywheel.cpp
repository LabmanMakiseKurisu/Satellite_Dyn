#include"Flywheel.h"

Flywheel::Flywheel()
{
	InstallVet << 1, 0, 0;
	Speed= 0;
	SpeedRef = 0;
	Torque = 0;
	TorqueRef = 0;
	LastRenewTime = 0;
	Kp = 1;
	Ki = 1;
	tau = 1;
	J = 0.064;
	MaxSpeed = 628;
	MaxTref = 0.5;
	Momentum = J * Speed;
}

Flywheel::Flywheel(Eigen::Vector3d& InsVet) :Flywheel()
{
	InstallVet = InsVet;
}

void Flywheel::StateRenew(int64_t NowTime, double SampleTime, double Tref)
{
	static double lastspeed = 0;
	static double lastspeedRef=0;

	//����ο�����
	TorqueRef = SATURATION(Tref, MaxTref);
	//����ο�ת��
	SpeedRef = TorqueRef * SampleTime / J + lastspeedRef;
	lastspeedRef = SpeedRef;
	//����ת��
	Speed = (1 - Kp / tau - Ki * SampleTime / tau) * lastspeed + (Kp / tau + Ki * SampleTime / tau) * lastspeedRef;
	Speed = SATURATION(Speed, MaxSpeed);
	//��������
	Torque = J * (Speed - lastspeed) / SampleTime;
	//����Ƕ���
	Momentum = J * Speed;
	//��һ��ת�ٸ���
	lastspeed = Speed;
}

void Flywheel::Init(double speed, int64_t timestamp)
{
	LastRenewTime = timestamp;
	Speed = speed;
	Momentum = J * Speed;
}



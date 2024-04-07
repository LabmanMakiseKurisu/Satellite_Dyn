#pragma once
#include"BaseMath.h"
#include"AllHead.h"
#include"Quaternions.h"
#include"Dcm.h"
#include"EulerAgl.h"
#include "Orbit.h"

class CAttitude
{
public:

	Eigen::Vector3d Omega_b;//����ϵ���ٶȣ���λrad/s
	CDcm Aio;//����ϵת���ϵת�ƾ���
	Quat Qib;//����ϵ������ϵ��Ԫ��
	Quat Qob;//���ϵ������ϵ��Ԫ��
private:
	Eigen::Matrix3d SatInaMat;//����ϵ�������󣬵�λkgm2
	Eigen::Vector3d WheelMomentum_b;//�������ڱ���ϵ�µĽǶ�������λNms
	Eigen::Vector3d TotalTorque;//Tf���������أ�TB �����أ�Tw�����ֱ���ϵ���� TotalTorque=TB+Tf-Tw
public:
	//
	// brief  : Ĭ����̬�๹�캯��
	//
	CAttitude();

	//
	// brief  : ��̬����ѧ���ƽ��ٶ�
	//
	int AttitudeDynamicsRk4(double Ts);

	//
	// brief  : ��̬�˶�������Ԫ��
	//
	int AttitudeKinematics(double Ts);

	void StateRenew(double Ts, COrbit& Orbit, CComponet* pComponet);

	void Init(COrbit& Obt);

	// д�����ݿ�
	void record(CInfluxDB& DB);

	static CDcm GetAio(const RV& InlRv);
private:
	Eigen::Vector3d LastOmega_b;//��һ�ĵı���ϵ���ٶȣ���λrad/s
	void RenewAio(COrbit& Orbit);
};



#pragma once
#include"BaseMath.hh"
#include"AllHead.hh"


class CAttitudeController
{
public:
	enum Mode
	{
		RATEDAMP,
		SUNPOINT,
		EARTHPOINT
	};
	Mode workmode;//��̬����ģʽ
	Eigen::Vector3d TorqueRef;//�ο�����Nm
	Eigen::Matrix3d Kp;//������ϵ��
	Eigen::Matrix3d Kd;//������ϵ��
	double MaxTorque;//�������
public:
	CAttitudeController();


	void record(CInfluxDB& DB);
	Eigen::Vector3d TorqueRefRenew(CComponet* pCom);
private:

	//@brief: �������������
	void RateDamping(const GyroScope& _Gyro);

	//@brief: ������̬������
	void ToSunControl(const GyroScope& _Gyro, const SunSensor& _Sun);

	//@brief: �Եز����붨������ʿ�����
	void ToEarthControl(const GyroScope& _Gyro, const StarSensor& _Star, const GNSS& _gnss);

};


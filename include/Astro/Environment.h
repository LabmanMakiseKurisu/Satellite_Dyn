#pragma once
#include"BaseMath.h"
// 2023-12-22 14:44:18
# include "AllHead.h"

class COrbit;
class CAttitude;
class Environment
{
public:
	Eigen::Vector3d BodyMag;//����ϵ�شų�ǿ��T
	Eigen::Vector3d NEDMag;//������ϵ�شų�ǿ��T
	Eigen::Vector3d SunVecInl;//����ϵ̫��ʸ��
	Eigen::Vector3d SunVecBody;//����ϵ̫��ʸ��

	Environment();

	//@brief: �������ϵת�ع�ϵ��ת�ƾ���
	//@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
	//@return : none
	static Eigen::Matrix3d ECI2ECEF(const int64_t timestamp, const double deltaUT1 = 0, const double xp = 0, const double yp = 0);

	//@brief: ����ϵ��̫��ʸ���ļ��㣨��ʱ���ת�������������н������ļ��㣩
	//@para :  timestamp: utcʱ���(ms)
	//@return : ����ϵ̫��ʸ��
	void SunPos(const int64_t timestamp);

	//@brief: ������ϵ�شų�
	//@para : �����������-�볤�� �����LLR
	//@return : none
	void GetNEDMag(const COrbit& Orbit, const int64_t timestamp);

	void StateRenew(CAttitude& Attitude, COrbit& Orbit, const int64_t timestamp);

	// д�����ݿ�
	void record(CInfluxDB& DB);
};
#pragma once
#include"BaseMath.h"
#include"Attitude.h"
#include"Orbit.h"
#include "Environment.h"
#include"Componet.h"
#include "AttitudeControl.h"


class CConfig
{
public:
	static CConfig* GetInstance();

public:
	int64_t SatelliteTime;//������utcʱ�����λms

	//���ﲻ��Eigen��Ϊ�˿ͻ��˺�����

	//��ʼ��������ݶ��ǹ���ϵ��
	double Rx, Ry, Rz;
	double Vx, Vy, Vz;

	//������
	double a;               //����볤��(Semi-major Axis)
	double e;               //���ƫ����(Eccentricity)
	double i;               //�����ǣ�rad�� (Inclination)
	double RAAN;            //������ྭ��rad��(RAAN)
	double omega;           //���ص���ǣ�rad��(Arg of Perigee)
	double M;               //���ƽ����ǣ�rad��(Mean Anomaly)

	//����ϵ���ٶ�
	double Wx, Wy, Wz;

	//��Ԫ�����ݶ�Qib
	double Q0, Q1, Q2, Q3;

	//��������
	double Jxx, Jxy, Jxz, Jyy, Jyz, Jzz;

	//�������ã� ��Ϊ����������������Ӧ�������ñ�

	//��������
	//�شŽ״Σ�1~12
	size_t MagOrder;
	//�شų���˹ϵ����һ���Զ������������ʹ��
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_g;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_h;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_gdot;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_hdot;
private:
	static inline CConfig* m_instance{ NULL };

	CConfig();
	~CConfig()=default;
	CConfig(const CConfig& _Config) = delete;
	CConfig& operator=(const CConfig& _Config) = delete;

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


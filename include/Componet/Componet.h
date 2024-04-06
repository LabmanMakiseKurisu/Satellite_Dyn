#pragma once
#include "Gyro.h"
#include"Flywheel.h"
#include"MagSensor.h"
#include"StarSensor.h"
#include"SunSensor.h"
#include "GNSS.h"
#include"AllHead.h"
// 2023-12-22 14:47:10

class CComponet
{
public:
	static CComponet* GetInstance();
	void Init(CAttitude& Att, COrbit& Obt, Environment& Env, CAttitudeController& ACtrl, int64_t timestamp);
	void StateRenew(CAttitude& Att, COrbit& Obt, Environment& Env, CAttitudeController& ACtrl, int64_t timestamp, double Ts);
	void record(CInfluxDB& DB);
public:
	size_t GyroNums;
	size_t FlywheelNums;
	size_t MagSensorNums;
	size_t StarSensorNums;
	size_t SunSensorNums;
	size_t GnssNums;
	std::vector<GyroScope> Gyros;
	std::vector<Flywheel> Wheels;
	std::vector<SunSensor> SunSensors;
	std::vector<StarSensor> StarSensors;
	std::vector<MagSensor> MagSensors;
	std::vector<GNSS> GNSSs;
private:
	static inline CComponet* m_instance{ NULL };

	CComponet();
	~CComponet();
	CComponet(const CComponet& _CComponet) = delete;
	CComponet& operator=(const CComponet& _CComponet) = delete;
	Eigen::VectorXd WheelsTrefCal(Eigen::Vector3d& TrefBody);

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

//��Componet��Ĺ��캯��������ɸ����������Ͳ����ĳ�ʼ��
//��Componet���Init��������ɸ������������ĳ�ʼ��
//��Componet���StateRenew��������ɸ����������ݸ���



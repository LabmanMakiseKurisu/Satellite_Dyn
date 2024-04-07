#include "AttitudeControl.hh"
#include"Dcm.hh"
#include"Quaternions.hh"
//#include"Orbit.hh"
#include"Attitude.hh"
#include"Componet.hh"
#include "Environment.hh"
#include"InfluxDB.hh"

CAttitudeController::CAttitudeController() :workmode(EARTHPOINT)
{
	TorqueRef << 0, 0, 0;
	Kp << 0.5 * Eigen::Matrix3d::Identity();
	Kd << 3 * Eigen::Matrix3d::Identity();
	MaxTorque = 0.08;
}

Eigen::Vector3d CAttitudeController::TorqueRefRenew(CComponet* pCom)
{
	switch (workmode)
	{
	case RATEDAMP:
	{
		RateDamping(pCom->Gyros[0]);
	}
	break;
	case SUNPOINT:
	{
		ToSunControl(pCom->Gyros[0], pCom->SunSensors[0]);
	}
	break;
	case EARTHPOINT:
	{

		ToEarthControl(pCom->Gyros[0], pCom->StarSensors[0], pCom->GNSSs[0]);
	}
	break;
	default:
		TorqueRef << 0, 0, 0;
		break;
	}
	return TorqueRef;
}

void CAttitudeController::RateDamping(const GyroScope& _Gyro)
{

	Eigen::Vector3d Tcontrol = -Kp * _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}

//���ղ����붨����������
void CAttitudeController::ToSunControl(const GyroScope& _Gyro, const SunSensor& _Sun)
{
	Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;

	//�ο���
	Eigen::Vector3d Wref(0, 0, 0.1 * DEG2RAD);
	Eigen::Vector3d Rb(0, 0, 1);
	Eigen::Vector3d _SunPos = _Sun.InstallMatrix.inverse() * _Sun.Data;
	//�������ؼ���
	Eigen::Vector3d Tcontrol = -Kp * _SunPos.cross(Rb) + Kd * (Eigen::Vector3d::Zero() - Wbi);
	//�޷�����
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}

//�Եز����붨����������
void CAttitudeController::ToEarthControl(const GyroScope& _Gyro, const StarSensor& _Star, const GNSS& _gnss)
{
	Eigen::Vector3d Wbi = _Gyro.InstallMatrix.inverse() * DEG2RAD * _Gyro.Data;
	Quat Qib = _Star.Data * _Star.InstallMatrix.ToQuat();

	//����Aio
	CDcm Aio = CAttitude::GetAio(_gnss.Data);

	Quat Qoi = Aio.ToQuat().QuatInv();
	Quat Qob = Qoi * Qib;
	Eigen::Vector3d ImQob;
	ImQob << Qob.QuatData[1], Qob.QuatData[2], Qob.QuatData[3];

	//�������ؼ���
	Eigen::Vector3d Tcontrol = -Kp * ImQob - Kd * Wbi;
	//�޷�����
	for (int i = 0; i < 3; i++)
	{
		Tcontrol[i] = LIMIT(Tcontrol[i], -MaxTorque, MaxTorque);
	}
	TorqueRef = Tcontrol;
}

void CAttitudeController::record(CInfluxDB& DB)
{	
	// ����ģʽ��InfluxDB��tag��ǩ���洢�ַ������ݣ�field�ֶΣ��洢��ֵ����
	DB.addKeyValue("SIM093", workmode);
	// ��������
	DB.addKeyValue("SIM094", TorqueRef.x());
	DB.addKeyValue("SIM095", TorqueRef.y());
	DB.addKeyValue("SIM096", TorqueRef.z());
}
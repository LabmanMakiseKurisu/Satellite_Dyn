#include"Attitude.h"
#include"CConfig.h"
#include"InfluxDB.h"

//��̬�˶�ѧ��������Ԫ��
#if 0
Eigen::Vector3d Omega_bRK4(Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Omega_b, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s, double Ts)
{
    Eigen::Vector3d k1, k2, k3, k4;
    k1 = AttDynamics(Omega_b, SatInaMat, Hw, Tau_s);
    k2 = AttDynamics(Omega_b + k1 * (0.5 * Ts), SatInaMat, Hw, Tau_s);
    k3 = AttDynamics(Omega_b + k2 * (0.5 * Ts), SatInaMat, Hw, Tau_s);
    k4 = AttDynamics(Omega_b + k3 * Ts, SatInaMat, Hw, Tau_s);

    return Omega_b + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
}

Quat AttKinematics(Quat Quat_k, Eigen::Vector3d Omega_b)
{
    double w, x, y, z;
    w = (-Omega_b(0,0) *Quat_k.GetByIdx(1) - Omega_b(1,0) * Quat_k.GetByIdx(2) - Omega_b(2,0) * Quat_k.GetByIdx(3)) * 0.5;
    x = (Omega_b(0,0) *Quat_k.GetByIdx(0) - Omega_b(1,0) * Quat_k.GetByIdx(3) + Omega_b(2,0) * Quat_k.GetByIdx(2)) * 0.5;
    y = (Omega_b(1,0) *Quat_k.GetByIdx(0) - Omega_b(2,0) *Quat_k.GetByIdx(1) + Omega_b(0,0) * Quat_k.GetByIdx(3)) * 0.5;
    z = (Omega_b(2,0) *Quat_k.GetByIdx(0) + Omega_b(1,0) *Quat_k.GetByIdx(1) - Omega_b(0,0) * Quat_k.GetByIdx(2)) * 0.5;
    return Quat(w, x, y, z);
}


Quat QibRK4(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts)
{
    Quat k1, k2, k3, k4,Res;
    k1 = AttKinematics(Quat_k, Omega_b);
    k2 = AttKinematics(Quat_k + k1 * (0.5 * Ts), Omega_b);
    k3 = AttKinematics(Quat_k + k2 * (0.5 * Ts), Omega_b);
    k4 = AttKinematics(Quat_k + k3 * Ts, Omega_b);
    Res = Quat_k + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
    return Res.QuatNormalize();
}

Quat QibIntegrate(Quat& Quat_k, Eigen::Vector3d& Omega_b, double Ts)
{
    Quat QuatTemp;
    QuatTemp = PlstToDeltaQuat(Omega_b, Ts);
    return Quat_k * QuatTemp;
}
#endif

//��̬����ѧ������ٶȱ仯��
Eigen::Vector3d AttDynamics(Eigen::Vector3d Omega_b, Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s)
{
    Eigen::Vector3d tmp = Omega_b.cross(SatInaMat * Omega_b + Hw);
    Eigen::Vector3d result = SatInaMat.inverse() * (Tau_s - tmp);
    return result;
}


//���ٶȺ�ʱ�����������Ԫ��
Quat PlstToDeltaQuat(const Eigen::Vector3d Omega_b, double OfstSec)
{
    double PlstVal = Omega_b.norm();
    return Quat(PlstVal * OfstSec, Omega_b);
}



CAttitude::CAttitude() :Qib(),Qob(),Aio()
{
    Omega_b << 0.05, -0.04, 0.1;
    LastOmega_b = Omega_b;
    SatInaMat << 10, 0, 0,
        0, 20, 0,
        0, 0, 30;
    WheelMomentum_b << 0, 0, 0;
    TotalTorque << 0, 0, 0;
}

int CAttitude::AttitudeDynamicsRk4(double Ts)
{
    Eigen::Vector3d k1, k2, k3, k4;
    k1 = AttDynamics(LastOmega_b, SatInaMat, WheelMomentum_b, TotalTorque);
    k2 = AttDynamics(LastOmega_b + k1 * (0.5 * Ts), SatInaMat, WheelMomentum_b, TotalTorque);
    k3 = AttDynamics(LastOmega_b + k2 * (0.5 * Ts), SatInaMat, WheelMomentum_b, TotalTorque);
    k4 = AttDynamics(LastOmega_b + k3 * Ts, SatInaMat, WheelMomentum_b, TotalTorque);
    Omega_b = LastOmega_b + (k1 + k2 * 2 + k3 * 2 + k4) * (Ts / 6);
    LastOmega_b = Omega_b;
    return 0;
}

int CAttitude::AttitudeKinematics(double Ts)
{
    Quat QuatTemp;
    QuatTemp = PlstToDeltaQuat(LastOmega_b, Ts);
    Qib = Qib * QuatTemp;
    return 0;
}

void CAttitude::RenewAio(COrbit& Orbit)
{
    Aio = CAttitude::GetAio(Orbit.J2000Inertial);
}

void CAttitude::StateRenew(double Ts, COrbit& Orbit, CComponet* pComponet)
{

    AttitudeKinematics(Ts);

    WheelMomentum_b << 0, 0, 0;
    TotalTorque << 0, 0, 0;
    for (size_t i = 0; i < pComponet->FlywheelNums; i++)
    {
        //������ֱ���ϵ�µĶ���
        WheelMomentum_b += pComponet->Wheels[i].InstallVet * pComponet->Wheels[i].Momentum;
        //������ֱ���ϵ�µ�����
        TotalTorque -= pComponet->Wheels[i].InstallVet * pComponet->Wheels[i].Torque;
    }
    AttitudeDynamicsRk4(Ts);

    RenewAio(Orbit);

    Quat Qio = Aio.ToQuat();
    Qob = Qio.QuatInv() * Qib;
}

void CAttitude::Init(COrbit& Obt)
{
    CConfig* pCfg = CConfig::GetInstance();
   Omega_b << pCfg->Wx, pCfg->Wy, pCfg->Wz;
   LastOmega_b = Omega_b;
   Qib.QuatData[0] = pCfg->Q0;
   Qib.QuatData[1] = pCfg->Q1;
   Qib.QuatData[2] = pCfg->Q2;
   Qib.QuatData[3] = pCfg->Q3;
   SatInaMat << pCfg->Jxx, pCfg->Jxy, pCfg->Jxz,
        pCfg->Jxy, pCfg->Jyy, pCfg->Jyz,
        pCfg->Jxz, pCfg->Jyz, pCfg->Jzz;
    RenewAio(Obt);
    Qob = Aio.ToQuat().QuatInv() * Qib;
}

void CAttitude::record(CInfluxDB& DB) {
    // ����ϵ������ϵ��Ԫ��
    DB.addKeyValue("SIM029", Qib.QuatData[0]);
    DB.addKeyValue("SIM030", Qib.QuatData[1]);
    DB.addKeyValue("SIM031", Qib.QuatData[2]);
    DB.addKeyValue("SIM032", Qib.QuatData[3]);
    // ���ϵ������ϵ��Ԫ��
    DB.addKeyValue("SIM033", Qob.QuatData[0]);
    DB.addKeyValue("SIM034", Qob.QuatData[1]);
    DB.addKeyValue("SIM035", Qob.QuatData[2]);
    DB.addKeyValue("SIM036", Qob.QuatData[3]);
    // ����ϵ���ٶ�
    DB.addKeyValue("SIM037", Omega_b.x() * RAD2DEG);
    DB.addKeyValue("SIM038", Omega_b.y() * RAD2DEG);
    DB.addKeyValue("SIM039", Omega_b.z() * RAD2DEG);


}

CDcm CAttitude::GetAio(const RV& InlRv)
{
    Eigen::Vector3d Pos = InlRv.Pos;//���ǵ�λ��ʸ��
    Eigen::Vector3d Vel = InlRv.Vel;//���ǵ��ٶ�ʸ��
    Eigen::Vector3d zo = Eigen::Vector3d::Zero() - Pos / Pos.norm();//ƫ���ᵥλʸ��
    Eigen::Vector3d y_tmp = Vel.cross(Pos);
    Eigen::Vector3d yo = y_tmp / y_tmp.norm(); // �����ᵥλʸ��
    Eigen::Vector3d xo = yo.cross(zo);//�����ᵥλʸ��


    CDcm Aio;
    Aio.DcmData << xo.transpose(), yo.transpose(), zo.transpose();
    return Aio;
}


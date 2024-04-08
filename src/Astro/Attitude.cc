#include"Attitude.hh"
#include"InfluxDB.hh"
#include"GlobalSetting.hh"
#include"Com_Schedule.hh"
#include"Quaternions.hh"
#include"Dcm.hh"
#include"EulerAgl.hh"
#include"Quaternions.hh"
#include "Orbit.hh"
#include "Flywheel.hh"
Eigen::Vector3d AttDynamics(Eigen::Vector3d Omega_b, Eigen::Matrix3d& SatInaMat, Eigen::Vector3d& Hw, Eigen::Vector3d& Tau_s)
{
    Eigen::Vector3d tmp = Omega_b.cross(SatInaMat * Omega_b + Hw);
    Eigen::Vector3d result = SatInaMat.inverse() * (Tau_s - tmp);
    return result;
}


//
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

void CAttitude::StateRenew(double Ts, COrbit& Orbit, Com_Schedule* pComponet)
{

    AttitudeKinematics(Ts);

    WheelMomentum_b << 0, 0, 0;
    TotalTorque << 0, 0, 0;
    for (size_t i = 0; i < pComponet->FlywheelNums; i++)
    {
        //
        WheelMomentum_b += pComponet->Wheels[i]->GetInsVet()*pComponet->Wheels[i]->GetMomentum();
        //
        TotalTorque -= pComponet->Wheels[i]->GetInsVet()*pComponet->Wheels[i]->GetTorque();
    }
    AttitudeDynamicsRk4(Ts);

    RenewAio(Orbit);

    Quat Qio = Aio.ToQuat();
    Qob = Qio.QuatInv() * Qib;
}

void CAttitude::Init(COrbit &Obt)
{
    GlobalSettings *pCfg = GlobalSettings::GetInstance();
    Omega_b << pCfg->Get<double>("/Omega_b/Wx"),
        pCfg->Get<double>("/Omega_b/Wy"),
        pCfg->Get<double>("/Omega_b/Wz");
    LastOmega_b = Omega_b;
    Qib.QuatData[0] = pCfg->Get<double>("/Qib/Q0");
    Qib.QuatData[1] = pCfg->Get<double>("/Qib/Q1");
    Qib.QuatData[2] = pCfg->Get<double>("/Qib/Q2");
    Qib.QuatData[3] = pCfg->Get<double>("/Qib/Q3");
    SatInaMat << pCfg->Get<double>("/SatInaMat/Jxx"), pCfg->Get<double>("/SatInaMat/Jxy"), pCfg->Get<double>("/SatInaMat/Jxz"),
        pCfg->Get<double>("/SatInaMat/Jxy"), pCfg->Get<double>("/SatInaMat/Jyy"), pCfg->Get<double>("/SatInaMat/Jyz"),
        pCfg->Get<double>("/SatInaMat/Jxz"), pCfg->Get<double>("/SatInaMat/Jyz"), pCfg->Get<double>("/SatInaMat/Jzz");

    RenewAio(Obt);
    Qob = Aio.ToQuat().QuatInv() * Qib;
    m_DM = DataManager::GetInstance(); 
    m_DM->Subscribe(this);
}

void CAttitude::Submit()
{
    if (!m_DM)
        return;
    int index = 1;
    for (int i = 0; i < 4; i++)
    {
        m_DM->add<double>(GetCode("SIM02",index), Qib.QuatData[i]);
        index++;
    }
    for (int i = 0; i < 4; i++)
    {
        m_DM->add<double>(GetCode("SIM02",index), Qob.QuatData[i]);
        index++;
    }
    for (int i = 0; i < 3; i++)
    {
        m_DM->add<double>(GetCode("SIM02",index), Omega_b[i]);
        index++;
    }
}

CDcm CAttitude::GetAio(const RV& InlRv)
{
    Eigen::Vector3d Pos = InlRv.Pos;//
    Eigen::Vector3d Vel = InlRv.Vel;//
    Eigen::Vector3d zo = Eigen::Vector3d::Zero() - Pos / Pos.norm();//
    Eigen::Vector3d y_tmp = Vel.cross(Pos);
    Eigen::Vector3d yo = y_tmp / y_tmp.norm(); // 
    Eigen::Vector3d xo = yo.cross(zo);//


    CDcm Aio;
    Aio.DcmData << xo.transpose(), yo.transpose(), zo.transpose();
    return Aio;
}


/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:01
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-11 18:56:49
 * @FilePath: /Satellite/src/Astro/Attitude.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include"Subscriber.hh"
#include"Dcm.hh"
#include"EulerAgl.hh"
#include"Quaternions.hh"
class Com_Schedule;
class COrbit;
struct RV;

class CAttitude : public ::ISubscriber
{
public:
	Quat Qib;//
	Quat Qob;//
	Eigen::Vector3d Omega_b;//rad/s
	CDcm Aio;//
private:
	Eigen::Matrix3d SatInaMat;//kgm2
	Eigen::Vector3d WheelMomentum_b;//Nms
	Eigen::Vector3d TotalTorque;//TotalTorque=TB+Tf-Tw
	std::string StartCode;
    int fileds;
public:
	//
	// brief  : 
	//
	CAttitude();
	//
	// brief  : 
	//
	int AttitudeDynamicsRk4(double Ts);
	//
	// brief  : 
	//
	int AttitudeKinematics(double Ts);

	void StateRenew(double Ts, COrbit& Orbit, Com_Schedule* pComponet);

	void Init(COrbit& Obt);

	virtual void Submit() override;

	static CDcm GetAio(const RV& InlRv);
private:
	Eigen::Vector3d LastOmega_b;//rad/s
	void RenewAio(COrbit& Orbit);
	double* Addr(int index);
};

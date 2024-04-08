/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:01
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 18:48:40
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
	Eigen::Vector3d Omega_b;//rad/s
	CDcm Aio;//
	Quat Qib;//
	Quat Qob;//
private:
	Eigen::Matrix3d SatInaMat;//kgm2
	Eigen::Vector3d WheelMomentum_b;//Nms
	Eigen::Vector3d TotalTorque;//TotalTorque=TB+Tf-Tw
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
};

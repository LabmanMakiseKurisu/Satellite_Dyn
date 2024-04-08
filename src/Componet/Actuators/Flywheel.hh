/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:11
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 18:52:02
 * @FilePath: /Satellite/src/Componet/Actuators/Flywheel.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include"Componet.hh"
struct WheelData
{
	Eigen::Vector3d InstallVet; //安装向量
	double intertia;			// 轮体惯量kgm2
	double Tau;					// 时间常数

	double Speed;				// 转速rad/s
	double Torque;				// 转矩Nm
	double Momentum;			// 角动量Nms
	double MaxSpeed;			// 最大转速rad/s
	double MaxTref;				// 最大转矩Nm

	double Kp;					// 控制器Kp
	double Ki;					// 控制器Ki
	double SpeedRef;			// 上一拍参考转速rad/s
	double TorqueRef;			// 上一拍参考转速Nm
	WheelData() = default;
	WheelData(Eigen::Vector3d &InsVet, 
	double _inertia, double _Tau, 
	double _MaxSpeed, double _MaxTref, 
	double _Kp, double _Ki) {
		InstallVet = InsVet;
		intertia = _inertia;
		Tau = _Tau;
		MaxSpeed = _MaxSpeed;
		MaxTref = _MaxTref;
		Kp = _Kp;
		Ki = _Ki;
		Momentum = intertia*Speed;
		SpeedRef = 0;
		TorqueRef = 0;
	}
	double operator[](size_t index) {
		switch (index)
		{
		case 0:
			return VEL2RPM(Speed);
		case 1:
			return Torque;
		default:
			return 0;
		}
	}
};
class Flywheel : public ::Componet<double, WheelData>
{
public:
	Flywheel(double *_s, Eigen::Vector3d &InsVet,
			 double _inertia, double _Tau,
			 double _MaxSpeed, double _MaxTref,
			 double _Kp, double _Ki,
			 int64_t timestamp, int _id,double Delta,
			 int _size = 2, std::string _StartCode = "SIM09");

public:
	virtual void Init(int64_t timestamp = 0);
	// 转矩模式
	virtual void StateRenew(int64_t NowTime = 0);
	Eigen::Vector3d &GetInsVet();
	double GetSpeed();
	double GetMomentum();
	double GetTorque();
	virtual void Submit() override;
private:
	double Delta;
};

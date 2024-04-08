/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:42
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 17:52:32
 * @FilePath: /Satellite/src/SatelliteMath/Quaternions.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
class CEulerAgl;
class CDcm;

class Quat
{
private:
	void QuatRectify();//
	void SelfNormalize();//

public:
	double QuatData[4];//JPL

public:
	Quat();//
	Quat(double q0, double q1, double q2, double q3);//
	Quat(double Theta, const Eigen::Vector3d& Axis);//
	Quat(const Quat& _Quat);//
	~Quat();

public:
	//
	Quat operator+(const Quat& _Quat) const;//
	Quat operator-(const Quat& _Quat) const;//
	Quat operator*(const Quat& _Quat) const;//
	Quat operator*(const double val) const;//
	Quat& operator=(const Quat& _Quat);
public:
	Quat QuatNormalize() const;//
	Quat QuatInv() const;//
	CDcm ToDcm() const;//
	CEulerAgl ToEulerAgl(unsigned Seq) const;//
};

std::ostream& operator<<(std::ostream& _cout, const Quat& _Quat);
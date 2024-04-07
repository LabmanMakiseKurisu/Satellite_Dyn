/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:42
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 19:52:39
 * @FilePath: /Satellite/src/SatelliteMath/Dcm.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
#include"AllHead.hh"

class CDcm
{
public:
	Eigen::Matrix3d DcmData;

public:

	CDcm();

	CDcm(double A00, double A01, double A02,
		 double A10, double A11, double A12,
		 double A20, double A21, double A22);
	

	CDcm(unsigned Axis, double Theta);


	CDcm(const CDcm& _Dcm);


	CDcm& operator=(const CDcm _Dcm);

public:

	CEulerAgl ToEulerAgl(unsigned Sequence) const;


	Quat ToQuat() const;

public:
	Eigen::Vector3d operator*(const Eigen::Vector3d& _Vector) const;//
	Eigen::Matrix3d operator*(const CDcm& _CDcm) const;//


};
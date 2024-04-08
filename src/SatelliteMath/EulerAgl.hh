/*
 * @Author: Amadeus
 * @Date: 2024-04-07 11:20:42
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-08 17:52:18
 * @FilePath: /Satellite/src/SatelliteMath/EulerAgl.hh
 * @Description: 
 */
#pragma once
#include"BaseMath.hh"
class Quat;
class CDcm;

class CEulerAgl
{
private:
	static bool CheckSeq(unsigned Seq);
public:
	struct {
		Eigen::Vector3d Angle;//(rad)
		unsigned Seq;//
	} AglData;//

public:
	CEulerAgl();

	CEulerAgl(double R1, double R2, double R3, unsigned Seq);

	CEulerAgl(const CEulerAgl& Agl);

	CEulerAgl& operator=(const CEulerAgl Agl);

public:
	CDcm ToDcm() const;

	Quat ToQuat() const;
};


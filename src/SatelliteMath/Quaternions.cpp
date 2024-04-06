#include"Quaternions.h"
#include"Dcm.h"
#include"EulerAgl.h"

void Quat::QuatRectify()
{
	if (QuatData[0] < 0)
		for (int i{ 0 }; i < 4; i++)
			QuatData[i] = -QuatData[i];
}

void Quat::SelfNormalize()
{
	double norm = sqrt(QuatData[0] * QuatData[0] + QuatData[1] * QuatData[1] + QuatData[2] * QuatData[2] + QuatData[3] * QuatData[3]);
	if (norm != 0)
	{
		double value = (1.0 / norm);
		for (int i{ 0 }; i < 4; i++)
			QuatData[i] *= value;
	}
}

Quat::Quat() :QuatData{ 1,0,0,0 }
{

}

Quat::Quat(double q0, double q1, double q2, double q3) :QuatData{ q0,q1,q2,q3 }
{
	QuatRectify();
	SelfNormalize();
}

Quat::Quat(const Quat& _Quat)
{
	for (int i{ 0 }; i < 4; i++)
		this->QuatData[i] = _Quat.QuatData[i];
	QuatRectify();
	SelfNormalize();
}

Quat::Quat(double Theta, const Eigen::Vector3d& Axis)
{
	Eigen::Vector3d axis_normalize = Axis.normalized();
	Theta = Theta * 0.5;
	double stheta = sin(Theta);
	QuatData[0] = cos(Theta);
	QuatData[1] = axis_normalize(0) * stheta;
	QuatData[2] = axis_normalize(1) * stheta;
	QuatData[3] = axis_normalize(2) * stheta;
	QuatRectify();
	SelfNormalize();
}

//Quat::Quat(const Eigen::Matrix3d& _Dcm)
//{
//	double trace = _Dcm(0,0) + _Dcm(1,1) + _Dcm(2,2);
//	if (trace > 0.0)
//	{
//		double sqtrp1 = sqrt(trace + 1.0);
//		QuatData[0] = (0.5 * sqtrp1);
//		QuatData[1] = ((_Dcm(1,2) - _Dcm(2,1)) / (2.0 * sqtrp1));
//		QuatData[2] = ((_Dcm(2,0) - _Dcm(0,2)) / (2.0 * sqtrp1));
//		QuatData[3] = ((_Dcm(0,1) - _Dcm(1,0)) / (2.0 * sqtrp1));
//	}
//	else
//	{
//		if (_Dcm(1,1) > _Dcm(0,0) && _Dcm(1,1) > _Dcm(2,2))
//		{
//			double sqtrp1 = sqrt(_Dcm(1,1) - _Dcm(0,0) - _Dcm(2,2) + 1.0);
//			QuatData[2] = (0.5 * sqtrp1);
//			if (sqtrp1!=0)
//			{
//				sqtrp1 = 0.5 / sqtrp1;
//			}
//			QuatData[0] = ((_Dcm(2,0) - _Dcm(0,2)) * sqtrp1);
//			QuatData[1] = ((_Dcm(0,1) + _Dcm(1,0)) * sqtrp1);
//			QuatData[3] = ((_Dcm(1,2) + _Dcm(2,1)) * sqtrp1);
//		}
//		else if (_Dcm(2,2) > _Dcm(0,0))
//		{
//			double sqtrp1 = sqrt(_Dcm(2,2) - _Dcm(0,0) - _Dcm(1,1) + 1.0);
//			QuatData[3] = (0.5 * sqtrp1);
//			if (sqtrp1!=0)
//			{
//				sqtrp1 = 0.5 / sqtrp1;
//			}
//			QuatData[0] = ((_Dcm(0,1) - _Dcm(1,0)) * sqtrp1);
//			QuatData[1] = ((_Dcm(2,0) + _Dcm(0,2)) * sqtrp1);
//			QuatData[2] = ((_Dcm(1,2) + _Dcm(2,1)) * sqtrp1);
//		}
//		else
//		{
//			double sqtrp1 = sqrt(_Dcm(0,0) - _Dcm(1,1) - _Dcm(2,2) + 1.0);
//			QuatData[1] = (0.5 * sqtrp1);
//			if (sqtrp1!=0)
//			{
//				sqtrp1 = 0.5 / sqtrp1;
//			}
//			QuatData[0] = ((_Dcm(1,2) - _Dcm(2,1)) * sqtrp1);
//			QuatData[2] = ((_Dcm(0,1) + _Dcm(1,0)) * sqtrp1);
//			QuatData[3] = ((_Dcm(2,0) + _Dcm(0,2)) * sqtrp1);
//		}
//	}
//	QuatRectify();
//	SelfNormalize();
//}

Quat Quat::operator+(const Quat& _Quat) const
{

	return Quat(QuatData[0] + _Quat.QuatData[0],
		QuatData[1] + _Quat.QuatData[1],
		QuatData[2] + _Quat.QuatData[2],
		QuatData[3] + _Quat.QuatData[3]);
}

Quat Quat::operator-(const Quat& _Quat) const
{
	return Quat(QuatData[0] - _Quat.QuatData[0],
		QuatData[1] - _Quat.QuatData[1],
		QuatData[2] - _Quat.QuatData[2],
		QuatData[3] - _Quat.QuatData[3]);
}

Quat Quat::operator*(const Quat& _Quat) const
{
	double w = QuatData[0] * _Quat.QuatData[0] - QuatData[1] * _Quat.QuatData[1] - QuatData[2] * _Quat.QuatData[2] - QuatData[3] * _Quat.QuatData[3];
	double x = QuatData[0] * _Quat.QuatData[1] + QuatData[1] * _Quat.QuatData[0] + QuatData[2] * _Quat.QuatData[3] - QuatData[3] * _Quat.QuatData[2];
	double y = QuatData[0] * _Quat.QuatData[2] - QuatData[1] * _Quat.QuatData[3] + QuatData[2] * _Quat.QuatData[0] + QuatData[3] * _Quat.QuatData[1];
	double z = QuatData[0] * _Quat.QuatData[3] + QuatData[1] * _Quat.QuatData[2] - QuatData[2] * _Quat.QuatData[1] + QuatData[3] * _Quat.QuatData[0];
	return Quat(w, x, y, z);
}

Quat Quat::operator*(const double val) const
{
	if (val != 0)
		return Quat(val * QuatData[0],
			val * QuatData[1],
			val * QuatData[2],
			val * QuatData[3]);
	else
	{
		printf("����ֵ�Ƿ� val:%f,����ԭ����Ԫ��\n", val);
		return *this;
	}
}

Quat& Quat::operator=(const Quat& _Quat)
{
	for (int i{ 0 }; i < 4; i++)
		QuatData[i] = _Quat.QuatData[i];
	QuatRectify();
	SelfNormalize();
	return *this;
}

Quat Quat::QuatNormalize() const
{
	double w, x, y, z;
	double norm = sqrt(QuatData[0] * QuatData[0] + QuatData[1] * QuatData[1] + QuatData[2] * QuatData[2] + QuatData[3] * QuatData[3]);
	if (norm != 0)
	{
		double value = (1.0 / norm);
		w = QuatData[0] * value;
		x = QuatData[1] * value;
		y = QuatData[2] * value;
		z = QuatData[3] * value;
		return Quat(w, x, y, z);
	}
	else
		return Quat();

}

Quat Quat::QuatInv() const
{

	return Quat(QuatData[0],
		-QuatData[1],
		-QuatData[2],
		-QuatData[3]);
}

CDcm Quat::ToDcm() const
{

	double ww = QuatData[0] * QuatData[0];
	double wx = QuatData[0] * QuatData[1];
	double wy = QuatData[0] * QuatData[2];
	double wz = QuatData[0] * QuatData[3];
	double xx = QuatData[1] * QuatData[1];
	double xy = QuatData[1] * QuatData[2];
	double xz = QuatData[1] * QuatData[3];
	double yy = QuatData[2] * QuatData[2];
	double yz = QuatData[2] * QuatData[3];
	double zz = QuatData[3] * QuatData[3];

	double A00 = ww + xx - yy - zz;
	double A01 = 2.0 * (xy + wz);
	double A02 = 2.0 * (xz - wy);
	double A10 = 2.0 * (xy - wz);
	double A11 = ww - xx + yy - zz;
	double A12 = 2.0 * (yz + wx);
	double A20 = 2.0 * (xz + wy);
	double A21 = 2.0 * (yz - wx);
	double A22 = ww - xx - yy + zz;

	return 	CDcm(A00, A01, A02,
		A10, A11, A12,
		A20, A21, A22);
}

CEulerAgl Quat::ToEulerAgl(unsigned Seq) const
{
	double ww = QuatData[0] * QuatData[0];
	double wx = QuatData[0] * QuatData[1];
	double wy = QuatData[0] * QuatData[2];
	double wz = QuatData[0] * QuatData[3];
	double xx = QuatData[1] * QuatData[1];
	double xy = QuatData[1] * QuatData[2];
	double xz = QuatData[1] * QuatData[3];
	double yy = QuatData[2] * QuatData[2];
	double yz = QuatData[2] * QuatData[3];
	double zz = QuatData[3] * QuatData[3];

	double R1 { 0 }, R2{ 0 }, R3{ 0 };

	switch (Seq)
	{
	case EUL_SQE_ZYX:
	{
		double r11 = 2 * (xy + wz);
		double r12 = ww + xx - yy - zz;
		double r21 = -2 * (xz - wy);
		double r31 = 2 * (yz + wx);
		double r32 = ww - xx - yy + zz;
		R1  = ATAN2(r11, r12);
		R2 = ASIN(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_ZYZ:
	{
		double r11 = 2 * (yz - wx);
		double r12 = 2 * (xz + wy);
		double r21 = ww - xx - yy + zz;
		double r31 = 2 * (yz + wx);
		double r32 = -2 * (xz - wy);
		R1  = ATAN2(r11, r12);
		R2 = ACOS(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_ZXY:
	{
		double r11 = -2 * (xy - wz);
		double r12 = ww - xx + yy - zz;
		double r21 = 2 * (yz + wx);
		double r31 = -2 * (xz - wy);
		double r32 = ww - xx - yy + zz;
		R1  = ATAN2(r11, r12);
		R2 = ASIN(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_ZXZ:
	{
		double r11 = 2 * (xz + wy);
		double r12 = -2 * (yz - wx);
		double r21 = ww - xx - yy + zz;
		double r31 = 2 * (xz - wy);
		double r32 = 2 * (yz + wx);
		R1  = ATAN2(r11, r12);
		R2 = ACOS(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_YXZ:
	{
		double r11 = 2 * (xz + wy);
		double r12 = ww - xx - yy + zz;
		double r21 = -2 * (yz - wx);
		double r31 = 2 * (xy + wz);
		double r32 = ww - xx + yy - zz;
		R1  = ATAN2(r11, r12);
		R2 = ASIN(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_YXY:
	{
		double r11 = 2 * (xy - wz);
		double r12 = 2 * (yz + wx);
		double r21 = ww - xx + yy - zz;
		double r31 = 2 * (xy + wz);
		double r32 = -2 * (yz - wx);
		R1  = ATAN2(r11, r12);
		R2 = ACOS(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_YZX:
	{
		double r11 = -2 * (xz - wy);
		double r12 = ww + xx - yy - zz;
		double r21 = 2 * (xy + wz);
		double r31 = -2 * (yz - wx);
		double r32 = ww - xx + yy - zz;
		R1  = ATAN2(r11, r12);
		R2 = ASIN(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_YZY:
	{

		double r11 = 2 * (yz + wx);
		double r12 = -2 * (xy - wz);
		double r21 = ww - xx + yy - zz;
		double r31 = 2 * (yz - wx);
		double r32 = 2 * (xy + wz);
		R1  = ATAN2(r11, r12);
		R2 = ACOS(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_XYZ:
	{
		double r11 = -2 * (yz - wx);
		double r12 = ww - xx - yy + zz;
		double r21 = 2 * (xz + wy);
		double r31 = -2 * (xy - wz);
		double r32 = ww + xx - yy - zz;
		R1  = ATAN2(r11, r12);
		R2 = ASIN(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_XYX:
	{
		double r11 = 2 * (xy + wz);
		double r12 = -2 * (xz - wy);
		double r21 = ww + xx - yy - zz;
		double r31 = 2 * (xy - wz);
		double r32 = 2 * (xz + wy);
		R1  = ATAN2(r11, r12);
		R2 = ACOS(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_XZY:
	{
		double r11 = 2 * (yz + wx);
		double r12 = ww - xx + yy - zz;
		double r21 = -2 * (xy - wz);
		double r31 = 2 * (xz + wy);
		double r32 = ww + xx - yy - zz;
		R1  = ATAN2(r11, r12);
		R2 = ASIN(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	case EUL_SQE_XZX:
	{
		double r11 = 2 * (xz - wy);
		double r12 = 2 * (xy + wz);
		double r21 = ww + xx - yy - zz;
		double r31 = 2 * (xz + wy);
		double r32 = -2 * (xy - wz);
		R1  = ATAN2(r11, r12);
		R2 = ACOS(r21);
		R3 = ATAN2(r31, r32);
	}
	break;
	default:
	{
		printf("sequence invalid\n");
	}
	break;
	}

	return CEulerAgl(R1, R2, R3, Seq);
}


std::ostream& operator<<(std::ostream& _cout, const Quat& _Quat)
{
	_cout << _Quat.QuatData[0] << " " << _Quat.QuatData[1] << " " << _Quat.QuatData[2] << " " << _Quat.QuatData[3] << std::endl;
	return _cout;
}

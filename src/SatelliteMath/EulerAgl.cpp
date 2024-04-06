#include"EulerAgl.h"
#include"Dcm.h"
#include"Quaternions.h"

bool CEulerAgl::CheckSeq(unsigned Seq)
{
	if (Seq == EUL_SQE_ZYZ ||
		Seq == EUL_SQE_ZXY ||
		Seq == EUL_SQE_ZYX ||
		Seq == EUL_SQE_ZXZ ||
		Seq == EUL_SQE_YXZ ||
		Seq == EUL_SQE_YXY ||
		Seq == EUL_SQE_YZX ||
		Seq == EUL_SQE_YZY ||
		Seq == EUL_SQE_XYZ ||
		Seq == EUL_SQE_XYX ||
		Seq == EUL_SQE_XZY ||
		Seq == EUL_SQE_XZX)
		return true;
	else
		return false;
}

CEulerAgl::CEulerAgl()
{
	AglData.Angle << 0, 0, 0;
	AglData.Seq = EUL_SQE_DEFAULT;
}

CEulerAgl::CEulerAgl(double R1, double R2, double R3, unsigned Seq) :CEulerAgl()
{
	if (CheckSeq(Seq))
	{
		AglData.Angle = Eigen::Vector3d(R1, R2, R3);
		AglData.Seq = Seq;
	}
	else
		printf("ת�򲻺Ϸ�������Ĭ��ŷ����\n");
}

CEulerAgl::CEulerAgl(const CEulerAgl& Agl) :CEulerAgl()
{
	if (CheckSeq(Agl.AglData.Seq))
	{
		*this = Agl; // ��������
	}
	else
		printf("ת�򲻺Ϸ�������Ĭ��ŷ����\n");
}

CEulerAgl& CEulerAgl::operator=(const CEulerAgl Agl)
{
	// ���ȼ���Ը�ֵ
	if (this != &Agl) {
		// ����ʵ�ʵĸ�ֵ����
		this->AglData.Angle = Agl.AglData.Angle;
		this->AglData.Seq = Agl.AglData.Seq;
	}
	return *this;
}

//ͨ������
CDcm CEulerAgl::ToDcm() const
{
	CDcm dcm;
	//��������������ת
	double psi = AglData.Angle(2); // ƫ���� 
	double theta = AglData.Angle(1); // ������  
	double phi = AglData.Angle(0); // ��ת�� 

	switch (AglData.Seq)
	{
	case EUL_SQE_ZYX:
	{
		double sx = sin(AglData.Angle(2));
		double sy = sin(AglData.Angle(1));
		double sz = sin(AglData.Angle(0));
		double cx = cos(AglData.Angle(2));
		double cy = cos(AglData.Angle(1));
		double cz = cos(AglData.Angle(0));
		dcm.DcmData(0, 0) = cy * cz;
		dcm.DcmData(0, 1) = cy * sz;
		dcm.DcmData(0, 2) = -sy;
		dcm.DcmData(1, 0) = sy * sx * cz - sz * cx;
		dcm.DcmData(1, 1) = sy * sx * sz + cz * cx;
		dcm.DcmData(1, 2) = cy * sx;
		dcm.DcmData(2, 0) = sy * cx * cz + sz * sx;
		dcm.DcmData(2, 1) = sy * cx * sz - cz * sx;
		dcm.DcmData(2, 2) = cy * cx;
	}
	break;
	case EUL_SQE_ZYZ:
	{
		double sy = sin(AglData.Angle(1));
		double sz = sin(AglData.Angle(0));
		double cy = cos(AglData.Angle(1));
		double cz = cos(AglData.Angle(0));
		double sz2 = sin(AglData.Angle(2));
		double cz2 = cos(AglData.Angle(2));
		dcm.DcmData(0, 0) = cz2 * cy * cz - sz2 * sz;
		dcm.DcmData(0, 1) = cz2 * cy * sz + sz2 * cz;
		dcm.DcmData(0, 2) = -cz2 * sy;
		dcm.DcmData(1, 0) = -sz2 * cy * cz - cz2 * sz;
		dcm.DcmData(1, 1) = -sz2 * cy * sz + cz2 * cz;
		dcm.DcmData(1, 2) = sz2 * sy;
		dcm.DcmData(2, 0) = sy * cz;
		dcm.DcmData(2, 1) = sy * sz;
		dcm.DcmData(2, 2) = cy;
	}
	break;
	case EUL_SQE_ZXY:
	{
		double sx = sin(AglData.Angle(1));
		double sy = sin(AglData.Angle(2));
		double sz = sin(AglData.Angle(0));
		double cx = cos(AglData.Angle(1));
		double cy = cos(AglData.Angle(2));
		double cz = cos(AglData.Angle(0));

		dcm.DcmData(0, 0) = cy * cz - sy * sx * sz;
		dcm.DcmData(0, 1) = cy * sz + sy * sx * cz;
		dcm.DcmData(0, 2) = -sy * cx;
		dcm.DcmData(1, 0) = -sz * cx;
		dcm.DcmData(1, 1) = cz * cx;
		dcm.DcmData(1, 2) = sx;
		dcm.DcmData(2, 0) = sy * cz + cy * sx * sz;
		dcm.DcmData(2, 1) = sy * sz - cy * sx * cz;
		dcm.DcmData(2, 2) = cy * cx;
	}
	break;
	case EUL_SQE_ZXZ:
	{
		double sx = sin(AglData.Angle(1));
		double sz = sin(AglData.Angle(0));
		double cx = cos(AglData.Angle(1));
		double cz = cos(AglData.Angle(0));
		double sz2 = sin(AglData.Angle(2));
		double cz2 = cos(AglData.Angle(2));

		dcm.DcmData(0, 0) = cz2 * cz - sz2 * cx * sz;
		dcm.DcmData(0, 1) = cz2 * sz + sz2 * cx * cz;
		dcm.DcmData(0, 2) = sz2 * sx;
		dcm.DcmData(1, 0) = -sz2 * cz - cz2 * cx * sz;
		dcm.DcmData(1, 1) = -sz2 * sz + cz2 * cx * cz;
		dcm.DcmData(1, 2) = cz2 * sx;
		dcm.DcmData(2, 0) = sz * sx;
		dcm.DcmData(2, 1) = -cz * sx;
		dcm.DcmData(2, 2) = cx;
	}
	break;
	case EUL_SQE_YXZ:
	{
		double sx = sin(AglData.Angle(1));
		double sy = sin(AglData.Angle(0));
		double sz = sin(AglData.Angle(2));
		double cx = cos(AglData.Angle(1));
		double cy = cos(AglData.Angle(0));
		double cz = cos(AglData.Angle(2));

		dcm.DcmData(0, 0) = cy * cz + sy * sx * sz;
		dcm.DcmData(0, 1) = sz * cx;
		dcm.DcmData(0, 2) = -sy * cz + cy * sx * sz;
		dcm.DcmData(1, 0) = -cy * sz + sy * sx * cz;
		dcm.DcmData(1, 1) = cz * cx;
		dcm.DcmData(1, 2) = sy * sz + cy * sx * cz;
		dcm.DcmData(2, 0) = sy * cx;
		dcm.DcmData(2, 1) = -sx;
		dcm.DcmData(2, 2) = cy * cx;
	}
	break;
	case EUL_SQE_YXY:
	{
		double sx = sin(AglData.Angle(1));
		double sy = sin(AglData.Angle(0));
		double cx = cos(AglData.Angle(1));
		double cy = cos(AglData.Angle(0));
		double sy2 = sin(AglData.Angle(2));
		double cy2 = cos(AglData.Angle(2));

		dcm.DcmData(0, 0) = cy2 * cy - sy2 * cx * sy;
		dcm.DcmData(0, 1) = sy2 * sx;
		dcm.DcmData(0, 2) = -cy2 * sy - sy2 * cx * cy;
		dcm.DcmData(1, 0) = sy * sx;
		dcm.DcmData(1, 1) = cx;
		dcm.DcmData(1, 2) = cy * sx;
		dcm.DcmData(2, 0) = sy2 * cy + cy2 * cx * sy;
		dcm.DcmData(2, 1) = -cy2 * sx;
		dcm.DcmData(2, 2) = -sy2 * sy + cy2 * cx * cy;
	}
	break;
	case EUL_SQE_YZX:
	{
		double sx = sin(AglData.Angle(2));
		double sy = sin(AglData.Angle(0));
		double sz = sin(AglData.Angle(1));
		double cx = cos(AglData.Angle(2));
		double cy = cos(AglData.Angle(0));
		double cz = cos(AglData.Angle(1));

		dcm.DcmData(0, 0) = cy * cz;
		dcm.DcmData(0, 1) = sz;
		dcm.DcmData(0, 2) = -sy * cz;
		dcm.DcmData(1, 0) = -sz * cx * cy + sy * sx;
		dcm.DcmData(1, 1) = cz * cx;
		dcm.DcmData(1, 2) = sy * cx * sz + cy * sx;
		dcm.DcmData(2, 0) = cy * sx * sz + sy * cx;
		dcm.DcmData(2, 1) = -cz * sx;
		dcm.DcmData(2, 2) = -sy * sx * sz + cy * cx;
	}
	break;
	case EUL_SQE_YZY:
	{
		double sy = sin(AglData.Angle(0));
		double sz = sin(AglData.Angle(1));
		double cy = cos(AglData.Angle(0));
		double cz = cos(AglData.Angle(1));
		double sy2 = sin(AglData.Angle(2));
		double cy2 = cos(AglData.Angle(2));

		dcm.DcmData(0, 0) = cy2 * cz * cy - sy2 * sy;
		dcm.DcmData(0, 1) = cy2 * sz;
		dcm.DcmData(0, 2) = -cy2 * cz * sy - sy2 * cy;
		dcm.DcmData(1, 0) = -cy * sz;
		dcm.DcmData(1, 1) = cz;
		dcm.DcmData(1, 2) = sy * sz;
		dcm.DcmData(2, 0) = sy2 * cz * cy + cy2 * sy;
		dcm.DcmData(2, 1) = sy2 * sz;
		dcm.DcmData(2, 2) = -sy2 * cz * sy + cy2 * cy;
	}
	break;
	case EUL_SQE_XYZ:
	{
		double sx = sin(AglData.Angle(0));
		double sy = sin(AglData.Angle(1));
		double sz = sin(AglData.Angle(2));
		double cx = cos(AglData.Angle(0));
		double cy = cos(AglData.Angle(1));
		double cz = cos(AglData.Angle(2));

		dcm.DcmData(0, 0) = cy * cz;
		dcm.DcmData(0, 1) = sz * cx + sy * sx * cz;
		dcm.DcmData(0, 2) = sz * sx - sy * cx * cz;
		dcm.DcmData(1, 0) = -cy * sz;
		dcm.DcmData(1, 1) = cz * cx - sy * sx * sz;
		dcm.DcmData(1, 2) = cz * sx + sy * cx * sz;
		dcm.DcmData(2, 0) = sy;
		dcm.DcmData(2, 1) = -cy * sx;
		dcm.DcmData(2, 2) = cy * cx;
	}
	break;
	case EUL_SQE_XYX:
	{
		double sx = sin(AglData.Angle(0));
		double sy = sin(AglData.Angle(1));
		double cx = cos(AglData.Angle(0));
		double cy = cos(AglData.Angle(1));
		double sx2 = sin(AglData.Angle(2));
		double cx2 = cos(AglData.Angle(2));

		dcm.DcmData(0, 0) = cy;
		dcm.DcmData(0, 1) = sy * sx;
		dcm.DcmData(0, 2) = -sy * cx;
		dcm.DcmData(1, 0) = sx2 * sy;
		dcm.DcmData(1, 1) = cx2 * cx - sx2 * cy * sx;
		dcm.DcmData(1, 2) = cx2 * sx + sx2 * cy * cx;
		dcm.DcmData(2, 0) = cx2 * sy;
		dcm.DcmData(2, 1) = -sx2 * cx - cx2 * cy * sx;
		dcm.DcmData(2, 2) = -sx2 * sx + cx2 * cy * cx;
	}
	break;
	case EUL_SQE_XZY:
	{
		double sx = sin(AglData.Angle(0));
		double sy = sin(AglData.Angle(2));
		double sz = sin(AglData.Angle(1));
		double cx = cos(AglData.Angle(0));
		double cy = cos(AglData.Angle(2));
		double cz = cos(AglData.Angle(1));

		dcm.DcmData(0, 0) = cy * cz;
		dcm.DcmData(0, 1) = sz * cx * cy + sy * sx;
		dcm.DcmData(0, 2) = cy * sx * sz - sy * cx;
		dcm.DcmData(1, 0) = -sz;
		dcm.DcmData(1, 1) = cz * cx;
		dcm.DcmData(1, 2) = cz * sx;
		dcm.DcmData(2, 0) = sy * cz;
		dcm.DcmData(2, 1) = sy * cx * sz - cy * sx;
		dcm.DcmData(2, 2) = sy * sx * sz + cy * cx;
	}
	break;
	case EUL_SQE_XZX:
	{
		double sx = sin(AglData.Angle(0));
		double sz = sin(AglData.Angle(1));
		double cx = cos(AglData.Angle(0));
		double cz = cos(AglData.Angle(1));
		double sx2 = sin(AglData.Angle(2));
		double cx2 = cos(AglData.Angle(2));

		dcm.DcmData(0, 0) = cz;
		dcm.DcmData(0, 1) = sz * cx;
		dcm.DcmData(0, 2) = sz * sx;
		dcm.DcmData(1, 0) = -cx2 * sz;
		dcm.DcmData(1, 1) = cx2 * cz * cx - sx2 * sx;
		dcm.DcmData(1, 2) = cx2 * cz * sx + sx2 * cx;
		dcm.DcmData(2, 0) = sx2 * sz;
		dcm.DcmData(2, 1) = -sx2 * cz * cx - cx2 * sx;
		dcm.DcmData(2, 2) = -sx2 * cz * sx + cx2 * cx;
	}
	break;
	}
	return dcm;
}

//ͨ������
Quat CEulerAgl::ToQuat() const
{
	// r fai     p  xita     y pusai  
	//����ת��
	//ת��1��2��3��
	double e1 = AglData.Angle[0] * 0.5;
	double e2 = AglData.Angle[1] * 0.5;
	double e3 = AglData.Angle[2] * 0.5;
	double s1 = sin(e1);
	double s2 = sin(e2);
	double s3 = sin(e3);
	double c1 = cos(e1);
	double c2 = cos(e2);
	double c3 = cos(e3);

	double W{ 1 }, X{ 0 }, Y{ 0 }, Z{ 0 };
	switch (AglData.Seq)
	{
	case EUL_SQE_ZYX:
	{
		W = c1 * c2 * c3 + s1 * s2 * s3;
		X = c1 * c2 * s3 - s1 * s2 * c3;
		Y = c1 * s2 * c3 + s1 * c2 * s3;
		Z = s1 * c2 * c3 - c1 * s2 * s3;
	}
	break;
	case EUL_SQE_ZYZ:
	{
		W = c1 * c2 * c3 - s1 * c2 * s3;
		X = c1 * s2 * s3 - s1 * s2 * c3;
		Y = c1 * s2 * c3 + s1 * s2 * s3;
		Z = s1 * c2 * c3 + c1 * c2 * s3;
	}
	break;
	case EUL_SQE_ZXY:
	{
		W = c1 * c2 * c3 - s1 * s2 * s3;
		X = c1 * s2 * c3 - s1 * c2 * s3;
		Y = c1 * c2 * s3 + s1 * s2 * c3;
		Z = c1 * s2 * s3 + s1 * c2 * c3;
		// ���ع������Ԫ��

	}break;
	case EUL_SQE_ZXZ:
	{
		W = c1 * c2 * c3 - s1 * c2 * s3;
		X = c1 * s2 * c3 + s1 * s2 * s3;
		Y = s1 * s2 * c3 - c1 * s2 * s3;
		Z = c1 * c2 * s3 + s1 * c2 * c3;
		// ���ع������Ԫ��


	}break;
	case EUL_SQE_YXZ:
	{
		W = c1 * c2 * c3 + s1 * s2 * s3;
		X = c1 * s2 * c3 + s1 * c2 * s3;
		Y = s1 * c2 * c3 - c1 * s2 * s3;
		Z = c1 * c2 * s3 - s1 * s2 * c3;
		// ���ع������Ԫ��

	}break;
	case EUL_SQE_YXY:
	{
		W = c1 * c2 * c3 - s1 * c2 * s3;
		X = c1 * s2 * c3 + s1 * s2 * s3;
		Y = s1 * c2 * c3 + c1 * c2 * s3;
		Z = c1 * s2 * s3 - s1 * s2 * c3;
		// ���ع������Ԫ��

	}break;
	case EUL_SQE_YZX:
	{
		W = c1 * c2 * c3 - s1 * s2 * s3;
		X = c1 * c2 * s3 + s1 * s2 * c3;
		Y = c1 * s2 * s3 + s1 * c2 * c3;
		Z = c1 * s2 * c3 - s1 * c2 * s3;
		// ���ع������Ԫ��

	}break;
	case EUL_SQE_YZY:
	{
		W = c1 * c2 * c3 - s1 * c2 * s3;
		X = s1 * s2 * c3 - c1 * s2 * s3;
		Y = c1 * c2 * s3 + s1 * c2 * c3;
		Z = c1 * s2 * c3 + s1 * s2 * s3;
		// ���ع������Ԫ��

	}break;
	case EUL_SQE_XYZ:
	{
		W = c1 * c2 * c3 - s1 * s2 * s3;
		X = c1 * s2 * s3 + s1 * c2 * c3;
		Y = c1 * s2 * c3 - s1 * c2 * s3;
		Z = c1 * c2 * s3 + s1 * s2 * c3;
		// ���ع������Ԫ��

	}break;
	case EUL_SQE_XYX:
	{
		W = c1 * c2 * c3 - s1 * c2 * s3;
		X = c1 * c2 * s3 + s1 * c2 * c3;
		Y = c1 * s2 * c3 + s1 * s2 * s3;
		Z = s1 * s2 * c3 - c1 * s2 * s3;
		// ���ع������Ԫ��

	}break;
	case EUL_SQE_XZY:
	{
		W = c1 * c2 * c3 + s1 * s2 * s3;
		X = s1 * c2 * c3 - c1 * s2 * s3;
		Y = c1 * c2 * s3 - s1 * s2 * c3;
		Z = c1 * s2 * c3 + s1 * c2 * s3;
		// ���ع������Ԫ��

	}break;
	case EUL_SQE_XZX:
	{
		W = c1 * c2 * c3 - s1 * c2 * s3;
		X = c1 * c2 * s3 + s1 * c2 * c3;
		Y = c1 * s2 * s3 - s1 * s2 * c3;
		Z = c1 * s2 * c3 + s1 * s2 * s3;
		// ���ع������Ԫ��

	}break;
	}
	return Quat(W, X, Y, Z);
}

#include"Dcm.hh"
#include"EulerAgl.hh"
#include"Quaternions.hh"


CDcm::CDcm() : DcmData(Eigen::Matrix3d::Identity())
{
    // ����
    // DcmData << 1, 0, 0, 0, 1, 0, 0, 0, 1;
}

CDcm::CDcm(double A00, double A01, double A02, double A10, double A11, double A12, double A20, double A21, double A22)
{
	/*�����Ż��β�*/
	DcmData << A00, A01, A02, 
		       A10, A11, A12, 
		       A20, A21, A22;

}

CDcm::CDcm(unsigned Axis, double Theta)
{
	//@brief: �Ե�����ת��ʼ��dcmdata
	//@para : axis(dcm_x_axis,dcm_y_axis,dcm_z_axis)��ת��(��basemath)
	//		  theta(rad)��ת��
	//@return : none
	//@remark : �Ѳ��� axis(dcm_x_axis=0?,dcm_y_axis=1?,dcm_z_axis=2?)
	switch (Axis)
	{
	case Dcm_X_AXIS:
	{
		DcmData << 1,      0,          0,
				   0,  cos(Theta), sin(Theta),
			       0, -sin(Theta), cos(Theta);
	}
	break;
	case Dcm_Y_AXIS:
	{
		DcmData << cos(Theta), 0, -sin(Theta),
			           0,      1,     0,
			       sin(Theta), 0, cos(Theta);
	}
	break;
	case Dcm_Z_AXIS:
	{
		DcmData << cos(Theta), sin(Theta), 0,
			      -sin(Theta), cos(Theta), 0,
			           0,          0,      1;
	}
	break;
	default:
	{
		printf("Axis invalid\n");
	}
	break;
	}
}


CDcm::CDcm(const CDcm& _Dcm)
{
    //@brief: �������������Ҿ����ʼ��DcmData��ע�����ﴫ��Ĳ������ͣ�Ҳ��CDcm
    //@para : _Dcm����һ���������Ҿ���
    //@return : none
    //@remark : �Ѳ���
    /*����*/
    DcmData = _Dcm.DcmData;
}

CDcm& CDcm::operator=(const CDcm _Dcm)
{
    //@brief: ���ظ�ֵ�����
    //@para : _Dcm����һ���������Ҿ���
    //@return : none
    //@remark : �Ѳ���
    /*���ظ�ֵ����� ���ص�ǰ���������*/
    if(this !=&_Dcm)
    DcmData = _Dcm.DcmData;
    return *this;
}

CEulerAgl CDcm::ToEulerAgl(unsigned Sequence) const
{
    //@brief:  �������Ҿ���תŷ����
    //@para : Sequence:ת��(��BaseMath)
    //@return : ָ��ת���ŷ����
    //@remark : �Ѳ���
    double R1{ 0 }, R2{ 0 }, R3{ 0 };

    switch (Sequence)
    {
    case EUL_SQE_ZYX:
    {
        R1 = ATAN2(DcmData(0, 1), DcmData(0, 0));
        R2 = ASIN(-DcmData(0, 2));
        R3 = ATAN2(DcmData(1, 2), DcmData(2, 2));
    }
    break;
    case EUL_SQE_ZYZ:
    {
        R1 = ATAN2(DcmData(2, 1), DcmData(2, 0));
        R2 = ACOS(DcmData(2, 2));
        R3 = ATAN2(DcmData(1, 2), -DcmData(0, 2));
    }
    break;
    case EUL_SQE_ZXY:
    {
        R1 = ATAN2(-DcmData(1, 0), DcmData(1, 1));
        R2 = ASIN(DcmData(1, 2));
        R3 = ATAN2(-DcmData(0, 2), DcmData(2, 2));
    }
    break;
    case EUL_SQE_ZXZ:
    {
        R1 = ATAN2(DcmData(2, 0), -DcmData(2, 1));
        R2 = ACOS(DcmData(2, 2));
        R3 = ATAN2(DcmData(0, 2), DcmData(1, 2));
    }
    break;
    case EUL_SQE_YXZ:
    {
        R1 = ATAN2(DcmData(2, 0), DcmData(2, 2));
        R2 = ASIN(-DcmData(2, 1));
        R3 = ATAN2(DcmData(0, 1), DcmData(1, 1));
    }
    break;
    case EUL_SQE_YXY:
    {
        R1 = ATAN2(DcmData(1, 0), DcmData(1, 2));
        R2 = ACOS(DcmData(1, 1));
        R3 = ATAN2(DcmData(0, 1), -DcmData(2, 1));
    }
    break;
    case EUL_SQE_YZX:
    {
        R1 = ATAN2(-DcmData(0, 2), DcmData(0, 0));
        R2 = ASIN(DcmData(0, 1));
        R3 = ATAN2(-DcmData(2, 1), DcmData(1, 1));
    }
    break;
    case EUL_SQE_YZY:
    {
        R1 = ATAN2(DcmData(1, 2), -DcmData(1, 0));
        R2 = ACOS(DcmData(1, 1));
        R3 = ATAN2(DcmData(2, 1), DcmData(0, 1));
    }
    break;
    case EUL_SQE_XYZ:
    {
        R1 = ATAN2(-DcmData(2, 1), DcmData(2, 2));
        R2 = ASIN(DcmData(2, 0));
        R3 = ATAN2(-DcmData(1, 0), DcmData(0, 0));
    }
    break;
    case EUL_SQE_XYX:
    {
        R1 = ATAN2(DcmData(0, 1), -DcmData(0, 2));
        R2 = ACOS(DcmData(0, 0));
        R3 = ATAN2(DcmData(1, 0), DcmData(2, 0));
    }
    break;
    case EUL_SQE_XZY:
    {
        R1 = ATAN2(DcmData(1, 2), DcmData(1, 1));
        R2 = ASIN(-DcmData(1, 0));
        R3 = ATAN2(DcmData(2, 0), DcmData(0, 0));
    }
    break;
    case EUL_SQE_XZX:
    {
        R1 = ATAN2(DcmData(0, 2), DcmData(0, 1));
        R2 = ACOS(DcmData(0, 0));
        R3 = ATAN2(DcmData(2, 0), -DcmData(1, 0));
    }
    break;
    default:
    {
        printf("sequence invalid\n");
    }
    break;
    }
    return CEulerAgl(R1, R2, R3, Sequence);
}

Quat CDcm::ToQuat() const
{
	//@brief:  �������Ҿ���ת��Ԫ��
	//@para : none
	//@return : ת���õ�����Ԫ��
	//@remark : �Ѳ���
	double trace = DcmData(0, 0) + DcmData(1, 1) + DcmData(2, 2);
	double q0{ 1 }, q1{ 0 }, q2{ 0 }, q3{ 0 };
	/*����*/
	if (trace > 0.0)
	{
		double sqtrp1 = sqrt(trace + 1.0);
		q0 = (0.5 * sqtrp1);
		q1 = ((DcmData(1, 2) - DcmData(2, 1)) / (2.0 * sqtrp1));
		q2 = ((DcmData(2, 0) - DcmData(0, 2)) / (2.0 * sqtrp1));
		q3 = ((DcmData(0, 1) - DcmData(1, 0)) / (2.0 * sqtrp1));
	}
	else
	{
		if (DcmData(1, 1) > DcmData(0, 0) && DcmData(1, 1) > DcmData(2, 2))
		{
			double sqtrp1 = sqrt(DcmData(1, 1) - DcmData(0, 0) - DcmData(2, 2) + 1.0);
			q2 = (0.5 * sqtrp1);
			if (sqtrp1 != 0)
			{
				sqtrp1 = 0.5 / sqtrp1;
			}
			q0 = ((DcmData(2, 0) - DcmData(0, 2)) * sqtrp1);
			q1 = ((DcmData(0, 1) + DcmData(1, 0)) * sqtrp1);
			q3 = ((DcmData(1, 2) + DcmData(2, 1)) * sqtrp1);
		}
		else if (DcmData(2, 2) > DcmData(0, 0))
		{
			double sqtrp1 = sqrt(DcmData(2, 2) - DcmData(0, 0) - DcmData(1, 1) + 1.0);
			q3 = (0.5 * sqtrp1);
			if (sqtrp1 != 0)
			{
				sqtrp1 = 0.5 / sqtrp1;
			}
			q0 = ((DcmData(0, 1) - DcmData(1, 0)) * sqtrp1);
			q1 = ((DcmData(2, 0) + DcmData(0, 2)) * sqtrp1);
			q2 = ((DcmData(1, 2) + DcmData(2, 1)) * sqtrp1);
		}
		else
		{
			double sqtrp1 = sqrt(DcmData(0, 0) - DcmData(1, 1) - DcmData(2, 2) + 1.0);
			q1 = (0.5 * sqtrp1);
			if (sqtrp1 != 0)
			{
				sqtrp1 = 0.5 / sqtrp1;
			}
			q0 = ((DcmData(1, 2) - DcmData(2, 1)) * sqtrp1);
			q2 = ((DcmData(0, 1) + DcmData(1, 0)) * sqtrp1);
			q3 = ((DcmData(2, 0) + DcmData(0, 2)) * sqtrp1);
		}
	}
	return Quat(q0, q1, q2, q3);
}

Eigen::Vector3d CDcm::operator*(const Eigen::Vector3d& _Vector) const
{
	return this->DcmData * _Vector;
}

Eigen::Matrix3d CDcm::operator*(const CDcm& _CDcm) const
{
	return this->DcmData * _CDcm.DcmData;
}
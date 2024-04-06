#pragma once
#include"BaseMath.h"
#include"AllHead.h"

class Quat
{
private:
	void QuatRectify();//��������������Ϊ��
	void SelfNormalize();//���ҹ�һ��

public:
	double QuatData[4];//JPL��Ԫ����ʵ����ǰ

public:
	Quat();//Ĭ�Ϲ��캯�������ص�λ��Ԫ��
	Quat(double q0, double q1, double q2, double q3);//��ֵ�ķ�ʽ������Ԫ��
	Quat(double Theta, const Eigen::Vector3d& Axis);//��ת��(rad)��ת�ᴴ����Ԫ��
	Quat(const Quat& _Quat);//��������Ԫ�����г�ʼ��
	~Quat()=default;

public:
	//���������
	Quat operator+(const Quat& _Quat) const;//��Ԫ���ӷ�
	Quat operator-(const Quat& _Quat) const;//��Ԫ������
	Quat operator*(const Quat& _Quat) const;//��Ԫ���˷�
	Quat operator*(const double val) const;//��Ԫ���˱���
	Quat& operator=(const Quat& _Quat);
public:
	Quat QuatNormalize() const;//��Ԫ����׼��
	Quat QuatInv() const;//��Ԫ������
	CDcm ToDcm() const;//��Ԫ��ת�������Ҿ���
	CEulerAgl ToEulerAgl(unsigned Seq) const;//��Ԫ��תָ��ת��ŷ����
};

std::ostream& operator<<(std::ostream& _cout, const Quat& _Quat);
#pragma once
#include"BaseMath.hh"
#include"AllHead.hh"

class CEulerAgl
{
private:
	static bool CheckSeq(unsigned Seq);
public:
	struct {
		Eigen::Vector3d Angle;//���������(rad)
		unsigned Seq;//ת��
	} AglData;//ŷ��������

public:
	//@brief: ��Ĭ��ת���0������ʼ��
	//@para : none
	//@return : none
	CEulerAgl();

	//@para : R1~R3(rad)������Ƕ�
	//		  Seq��ת��
	//@return : none
	CEulerAgl(double R1, double R2, double R3, unsigned Seq);

	//@brief: ������ŷ���ǳ�ʼ��
	//@para : Agl��ŷ����
	//@return : none
	CEulerAgl(const CEulerAgl& Agl);

	//@brief: ���ظ�ֵ�����
	//@para : Agl��ŷ����
	//@return : none
	CEulerAgl& operator=(const CEulerAgl Agl);

public:
	//@brief:  ŷ����ת�������Ҿ���
	//@para : Sequence:ת��(��BaseMath)
	//@return : �������Ҿ���
	CDcm ToDcm() const;

	//@brief:  ŷ����ת��Ԫ��
	//@para :  none
	//@return : ��Ԫ��
	Quat ToQuat() const;
};


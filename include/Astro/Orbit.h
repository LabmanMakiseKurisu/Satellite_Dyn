#pragma once
#include"BaseMath.h"
#include"AllHead.h"
// 2023-12-22 11:19:45
struct RV
{
    Eigen::Vector3d Pos;//�����λ��ʸ������λm
    Eigen::Vector3d Vel;//������ٶ�ʸ������λm/s
    RV() : Pos(6678136.9999998566, 0.0002095685, -1.3800009224), 
            Vel(0.0007615644, 6789.5304738682, 3686.4138485846) {}
    RV(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel)
     : Pos(initialPos),Vel(initialVel){}
};
struct LLA_t
{
    double Lng;//�������ȣ���λ����
    double Lat;//����γ�ȣ���λ����
    double Alt;//���θ߶ȣ���λm
    LLA_t() : Lng(0), Lat(1.3963), Alt(0) {}
};
struct LLR_t
{
    double Lng;//���ľ��ȣ���λ����
    double Lat;//����γ�ȣ���λ����
    double Rds;//���İ뾶����λm
    LLR_t() : Lng(0), Lat(1.3951), Rds(6357400) {}
};

struct OrbitElement
{
    double a;               //����볤��(Semi-major Axis)
    double e;               //���ƫ����(Eccentricity)
    double i;               //�����ǣ�rad�� (Inclination)
    double RAAN;            //������ྭ��rad��(RAAN)
    double omega;           //���ص���ǣ�rad��(Arg of Perigee)
    double M;               //���ƽ����ǣ�rad��(Mean Anomaly)
    double f;               //�����ǣ�rad��(True Anomaly)
    double u;               //������ǣ�γ�ȷ��ǣ���rad��(Arg of Latitude)
    double E;               //���ƫ����ǣ�rad��(Eccentric Anomaly)
    double w;               //���ƽ�����ٶȣ�rad/s��(Palstance  2PI/Period)
    double T;               //������ڣ�s��(Period)
    OrbitElement():
        a(6678137), e(0), i(RAD(28.5)),
        RAAN(0), omega(0), M(0)
    {
        double Epre = M;
        double Enxt = 0.0;
        for (int i{0};i<5;i++)
        {
            Enxt = Epre - (Epre - e * sin(Epre) - M) / (1.0 - e * cos(Epre));
            if (fabs(Enxt - Epre) < 1e-7) { break; }
            Epre = Enxt;
        }
        E = RAD_2PI(Enxt);
        f = RAD_2PI(ATAN2(SQRT(1.0 - e * e) * sin(E), cos(E) - e));
        u = RAD_2PI(omega + f);
        w = sqrt(EARTH_GRAVITATIONAL / (pow(a, 3)));
        T = TWOPI / w;
    }
};

class COrbit
{
private:
    bool IsRV(RV& rv)
    {
        return (rv.Pos.norm() > EARTH_EQUATORIAL_RADIUS && rv.Vel.norm() > 0 && rv.Vel.norm() < 7900);
    }

    bool IsOrbitElement(OrbitElement& oe)
    {
        return (oe.a > EARTH_EQUATORIAL_RADIUS && oe.e >= 0 && oe.e < 1.0);
    }

public:
    RV J2000Inertial;//����ϵRV
    RV ECEFFix;//�ع�ϵRV
    OrbitElement OrbitElements;//�������
    LLA_t LLA;//��γ��
    LLR_t LLR;//����γ�ȺͰ뾶


public:
    COrbit(): J2000Inertial(), OrbitElements(), ECEFFix(), LLA(), LLR()
    {
        //��дһ���Թ���ϵ��ʼ���ķ�ʽ��

    }

    //
    // brief  : ʹ�ù���ϵRV�Ͷ�����ƹ��
    //
    int TwoBodRK4(double Ts);

    //@brief: ����ϵλ���ٶ�ת�ع�ϵλ���ٶ�
    //@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
    //@return : none
    //@remark : �Ѳ���
    void Inl2Fix(const int64_t timestamp);

    //@brief: �ع�ϵ�������LLA
    //@para : none
    //@return : none
    //@remark : �Ѳ���
    void FixPos2LLA();

    //@brief: �ع�ϵ�������LLR
    //@para : none
    //@return : none
    //@remark : �Ѳ���
    void FixPos2LLR();

    //@brief: ���㱱����ϵ���ع�ϵ��ת�ƾ���
    //@para : timestamp: utcʱ���(ms) deltaUT1:UTC-UT1(s) xp,yp:����(rad)  rc2t:ת�ƾ�����
    //@return : none
    //@remark : ��̬��Ա����ֻ�ܷ��ʾ�̬��Ա����
    Eigen::Matrix3d NED2ECEF();

    void StateRenew(double Ts, const int64_t timestamp);

    void Init(int64_t Timestamp);

    // д�����ݿ�
    void record(CInfluxDB& DB);
private:
    Eigen::VectorXd TwoBodAcc(const Eigen::VectorXd& RVState);
};


std::ostream& operator<<(std::ostream& _cout, const RV& j2000);
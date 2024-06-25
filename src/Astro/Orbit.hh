#pragma once
#include"BaseMath.hh"
#include"Subscriber.hh"


struct RV
{
    Eigen::Vector3d Pos;//m
    Eigen::Vector3d Vel;//m/s
    RV() : Pos(6678136.9999998566, 0.0002095685, -1.3800009224), 
            Vel(0.0007615644, 6789.5304738682, 3686.4138485846) {}
    RV(const Eigen::Vector3d& initialPos, const Eigen::Vector3d& initialVel)
     : Pos(initialPos),Vel(initialVel){}
    double operator[](size_t index) {
        switch (index)
        {
        case 0:
            return Pos[0];
        case 1:
            return Pos[1];
        case 2:
            return Pos[2];
        case 3:
            return Vel[0];
        case 4:
            return Vel[1];
        case 5:
            return Vel[2];
        default:
            return 0;
        }
    }
    double* Addr(size_t index) {
        switch (index)
        {
        case 0:
            return &Pos[0];
        case 1:
            return &Pos[1];
        case 2:
            return &Pos[2];
        case 3:
            return &Vel[0];
        case 4:
            return &Vel[1];
        case 5:
            return &Vel[2];
        default:
            return nullptr;
        }
    }
    int size() { return 6; }
};
struct LLA_t
{
    double Lng; // rad
    double Lat; // rad
    double Alt; // m
    LLA_t() : Lng(0), Lat(1.3963), Alt(0) {}
    double operator[](size_t index)
    {
        switch (index)
        {
        case 0:
            return RAD2DEG * Lng;
        case 1:
            return RAD2DEG * Lat;
        case 2:
            return Alt;
        default:
            return 0;
        }
    }
    double *Addr(size_t index)
    {
        switch (index)
        {
        case 0:
            return &Lng;
        case 1:
            return &Lat;
        case 2:
            return &Alt;
        default:
            return nullptr;
        }
    }
    int size() { return 3; }
};
struct LLR_t
{
    double Lng; // rad
    double Lat; // rad
    double Rds; // m
    LLR_t() : Lng(0), Lat(1.3951), Rds(6357400) {}
    double operator[](size_t index)
    {
        switch (index)
        {
        case 0:
            return RAD2DEG * Lng;
        case 1:
            return RAD2DEG * Lat;
        case 2:
            return Rds;
        default:
            return 0;
        }
    }
    double *Addr(size_t index)
    {
        switch (index)
        {
        case 0:
            return &Lng;
        case 1:
            return &Lat;
        case 2:
            return &Rds;
        default:
            return nullptr;
        }
    }
    int size() { return 3; }
};

struct OrbitElement
{
    double a;               //(Semi-major Axis)
    double e;               //(Eccentricity)
    double i;               //(Inclination)
    double RAAN;            //rad(RAAN)
    double omega;           //rad(Arg of Perigee)
    double M;               //rad(Mean Anomaly)
    double f;               //rad(True Anomaly)
    double u;               //ad(Arg of Latitude)
    double E;               //rad(Eccentric Anomaly)
    double w;               //rad/s(Palstance  2PI/Period)
    double T;               //s(Period)
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
    int size() { return 11; }
};

class COrbit: public ::ISubscriber
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
    RV J2000Inertial;//
    RV ECEFFix;//
    LLA_t LLA;//

public:
    OrbitElement OrbitElements;//
    LLR_t LLR;//
public:
    std::string StartCode;
    int fileds;

public:
    COrbit();
    //
    // brief  : 
    //
    int TwoBodRK4(double Ts);

    //@brief: 
    //@para : timestamp: utc(ms) deltaUT1:UTC-UT1(s) xp,yp:(rad)  rc2t:
    //@return : none
    void Inl2Fix(const int64_t timestamp);

    static RV Fix2Inl(const int64_t timestamp, const RV &fix);

    void FixPos2LLA();


    void FixPos2LLR();

    //@para : timestamp: utcʱ(ms) deltaUT1:UTC-UT1(s) xp,yp:(rad)  rc2t:
    Eigen::Matrix3d NED2ECEF();

    void StateRenew(double Ts, const int64_t timestamp);

    void Init(int64_t Timestamp);

    virtual void Submit() override;
private:
    Eigen::VectorXd TwoBodAcc(const Eigen::VectorXd& RVState);
    double *Addr(int index);
};


std::ostream& operator<<(std::ostream& _cout, const RV& j2000);
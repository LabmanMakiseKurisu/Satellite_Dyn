/*
 * @Author: Amadeus
 * @Date: 2024-04-07 15:40:13
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-07 16:44:45
 * @FilePath: /Satellite/src/General/GlobalSetting.hh
 * @Description: 
 */
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include"BaseMath.hh"

class GlobalSettings
{
public:
    static GlobalSettings *GetInstance();

public:
    template<typename T>
    T Get(const std::string& keyPath) const {
        nlohmann::json::json_pointer pointer(keyPath);
        return settings[pointer].get<T>();
    }
    Eigen::MatrixXd GetMat(const std::string& keyPath) const {
        nlohmann::json::json_pointer pointer(keyPath);
        const std::string str = settings[pointer].get<std::string>();
        return  ParseMatrix(str);
    }
    Eigen::VectorXd GetVec(const std::string& keyPath) const {
        nlohmann::json::json_pointer pointer(keyPath);
        std::string str = settings[pointer].get<std::string>();
        return ParseVector(str);
    }
    auto& Getgauss_g() const { return gauss_g; };
    auto& Getgauss_h() const { return gauss_h; };
    auto& Getgauss_gdot() const { return gauss_gdot; };
    auto& Getgauss_hdot() const { return gauss_hdot; };
private:
    static inline GlobalSettings* m_instance{ NULL };
    nlohmann::json settings;
    Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_g;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_h;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_gdot;
	Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic> gauss_hdot;

private:
    GlobalSettings(const std::string &configFilePath);
    ~GlobalSettings() = default;
    GlobalSettings(const GlobalSettings &_Config) = delete;
    static void ReleaseInstance();
    class DeleteHelper
    {
    public:
        DeleteHelper() = default;
        ~DeleteHelper()
        {
            ReleaseInstance();
        }
    };
    static DeleteHelper helper;
    GlobalSettings &operator=(const GlobalSettings &_Config) = delete;
    Eigen::MatrixXd ParseMatrix(const std::string &matrixString) const;
    Eigen::VectorXd ParseVector(const std::string &vectorString) const;
};
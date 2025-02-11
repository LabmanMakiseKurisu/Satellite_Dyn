/*
 * @Author: Amadeus
 * @Date: 2024-04-07 15:53:05
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-11 19:02:35
 * @FilePath: /Satellite/src/General/GlobalSetting.cc
 * @Description:
 */
#include "GlobalSetting.hh"

static GlobalSettings* m = GlobalSettings::GetInstance();
GlobalSettings::DeleteHelper GlobalSettings::helper;

GlobalSettings* GlobalSettings::GetInstance()
{
	if (m_instance == NULL)
		m_instance = new GlobalSettings("Config/GlobalSetting.json");
	return m_instance;
}

void GlobalSettings::ReleaseInstance()
{
	GlobalSettings* tmp = m_instance;
	m_instance = NULL;
	delete tmp;
}

void GlobalSettings::ParseGaussCof()
{
    int MagOrder = this->Get<int>("/Env/Mag/MagOrder");
    if ((MagOrder < 1) || (MagOrder > 12))
    {
        MagOrder = 2;
    }
    Eigen::Index rows = (MagOrder + 1) * (MagOrder + 2) / 2 - 1;
    gauss_g.resize(MagOrder + 1, MagOrder + 1);
    gauss_h.resize(MagOrder + 1, MagOrder + 1);
    gauss_gdot.resize(MagOrder + 1, MagOrder + 1);
    gauss_hdot.resize(MagOrder + 1, MagOrder + 1);
    gauss_g.setZero();
    gauss_h.setZero();
    gauss_gdot.setZero();
    gauss_hdot.setZero();

    std::ifstream file("Config/wmm_2020_data.cfg");
    if (file.is_open())
    {
        //
        int row, col;
        for (int i = 0; i < rows; i++)
        {
            // r c g h gdot hdot
            file >> row;
            file >> col;
            file >> gauss_g(row, col);
            file >> gauss_h(row, col);
            file >> gauss_gdot(row, col);
            file >> gauss_hdot(row, col);
        }
        file.close();
    }
    else
    {
        std::cerr << "Unable to open file wmm_2020_data" << std::endl;
        exit(0);
    }
}

void GlobalSettings::ParseStokesCof()
{
    int GravityOrder = this->Get<int>("/Env/Gravity/GravityOrder");
    if ((GravityOrder < 1) || (GravityOrder > 99))
    {
        GravityOrder = 2;
    }
    int rows = (GravityOrder + 1) * (GravityOrder + 2) / 2 - 1;
    stokes_c.resize(GravityOrder + 2, GravityOrder + 2);
    stokes_s.resize(GravityOrder + 2, GravityOrder + 2);
    stokes_c.setZero();
    stokes_s.setZero();

    std::ifstream file("Config/EGM2008.cfg");
    if (file.is_open())
    {
        int row, col;
        for (int i = 0; i < rows; i++)
        {
            file >> row;
            file >> col;
            file >> stokes_c(row, col);
            file >> stokes_s(row, col);
        }
        file.close();
    }
    else
    {
        std::cerr << "Unable to open file EGM2008" << std::endl;
        exit(0);
    }
}

void GlobalSettings::ParseF107Cof()
{
    std::ifstream file("Config/F107.json");
    if(!file.is_open()) {
        std::cerr << "Unable to open file F107.json" << std::endl;
        exit(0);
    }
    nlohmann::json j;
    file >> j;

    for (const auto& entry : j["hpcoef"]) {
        int f107_value = entry["F107"];
        for (const auto& d : entry["data"]) {
            Eigen::Vector3d density_data(d[0], d[1], d[2]);
            F107[f107_value].push_back(density_data);
        }
    }
    return;
}

GlobalSettings::GlobalSettings(const std::string &configFilePath) 
{
    std::ifstream configFile(configFilePath);
    if (configFile.is_open())
    {
        configFile >> settings;
    }
    else
    {
        std::cerr << "Unable to open config file: " << configFilePath << std::endl;
        exit(0);
    }
    ParseGaussCof();
    ParseStokesCof();
    ParseF107Cof();
    return;
}

Eigen::MatrixXd GlobalSettings::ParseMatrix(const std::string &matrixString) const
{
    std::vector<double> matrixValues;
    std::vector<int> colCounts;
    int colCount = 0;

    std::stringstream ss(matrixString);
    std::string rowString;

    //
    while (std::getline(ss, rowString, ';'))
    {
        std::stringstream rowStream(rowString);
        std::string cell;

        //
        while (std::getline(rowStream, cell, ','))
        {
            matrixValues.push_back(std::stod(cell));
            colCount++;
        }

        colCounts.push_back(colCount);
        colCount = 0;
    }

    //
    for (size_t i = 1; i < colCounts.size(); ++i)
    {
        if (colCounts[i] != colCounts[0])
        {
            throw std::runtime_error("Matrix rows have varying number of columns");
        }
    }

    //
    int rows = colCounts.size();
    int cols = colCounts.empty() ? 0 : colCounts[0];
    Eigen::MatrixXd matrix(rows, cols);

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            matrix(i, j) = matrixValues[i * cols + j];
        }
    }

    return matrix;
}

Eigen::VectorXd GlobalSettings::ParseVector(const std::string &vectorString) const
{
    std::vector<double> vectorValues;

    std::stringstream ss(vectorString);
    std::string cell;

    //
    while (std::getline(ss, cell, ','))
    {
        vectorValues.push_back(std::stod(cell));
    }

    //
    int size = vectorValues.size();
    Eigen::VectorXd vector(size);

    for (int i = 0; i < size; ++i)
    {
        vector[i] = vectorValues[i];
    }

    return vector;
}
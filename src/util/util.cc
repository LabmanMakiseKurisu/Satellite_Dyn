/*
 * @Author: Amadeus
 * @Date: 2024-04-12 10:43:55
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 10:54:17
 * @FilePath: /Satellite/src/util/util.cc
 * @Description: 
 */
#include<vector>
#include"util.hh"


Eigen::MatrixXd util::ParseMatrix(const std::string &matrixString) {
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

Eigen::VectorXd util::ParseVector(const std::string &vectorString)
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

std::string util::GetCode(std::string StartCode, int index)
{
    std::string Code = StartCode;
    if (index < 10)
        Code.append("0");
    Code.append(std::to_string(index));
    return Code;
}
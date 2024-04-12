/*
 * @Author: Amadeus
 * @Date: 2024-04-12 10:43:48
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 10:57:00
 * @FilePath: /Satellite/src/util/util.hh
 * @Description: 
 */
#pragma once
#include<iostream>
#include"BaseMath.hh"

class util
{
public:
    static Eigen::MatrixXd ParseMatrix(const std::string &matrixString);
    static Eigen::VectorXd ParseVector(const std::string &vectorString);
    static std::string GetCode(std::string StartCode, int index);
};
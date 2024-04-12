/*
 * @Author: Amadeus
 * @Date: 2024-04-11 18:31:10
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 12:09:21
 * @FilePath: /Satellite/src/api/Mediator.hh
 * @Description: 
 */
#pragma once
#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>
#include<unordered_map>

class Mediator
{
public:
    enum class ErrorType
    {
        NO_ERROR,
        INVALID_KEY,
        UNSUPPORTED_TYPE
    };
    static Mediator *GetInstance();

    void add(std::string key, double *value);
    ErrorType get(std::string &res, std::string key, std::string type, int fileds = 1);
    ErrorType set(std::string &val, std::string key, std::string type, int fileds = 1);
    static std::string ErrorTypeStr(ErrorType type);

private:
    static inline Mediator *m_instance{NULL};
    std::unordered_map<std::string, double *> data;

private:
    Mediator();
    Mediator(const Mediator &) = delete;
    Mediator &operator=(const Mediator &) = delete;
    Mediator(Mediator &&) = delete;
    Mediator &operator=(Mediator &&) = delete;
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
};

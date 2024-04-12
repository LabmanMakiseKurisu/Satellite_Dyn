/*
 * @Author: Amadeus
 * @Date: 2024-04-07 18:01:03
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-12 17:06:52
 * @FilePath: /Satellite/src/api/Mediator.cc
 * @Description: 
 */
#include"Mediator.hh"
#include"util.hh"
#include"BaseMath.hh"
static Mediator* m = Mediator::GetInstance();
Mediator::DeleteHelper Mediator::helper;

Mediator *Mediator::GetInstance()
{
    if (m_instance == NULL)
        m_instance = new Mediator;
    return m_instance;
}

void Mediator::add(std::string key, double *value)
{
    std::cout << "Handler: " << key << " is " << value << std::endl;
    if (value == nullptr)
    {
        exit(0);
    }
    data[key] = value;
}

Mediator::ErrorType Mediator::get(std::string &res, std::string key, std::string type, int fileds)
{
    if (type == "number")
    {
        if (data.find(key) == data.end())
            return ErrorType::INVALID_KEY;
        res = std::to_string(*data[key]);
        return ErrorType::NO_ERROR;
    }
    else if (type == "vector")
    {
        res.clear();
        for (int i = 0; i < fileds; i++)
        {
            std::string code = util::GetCode(key, i);
            if (data.find(code) != data.end())
            {
                res += std::to_string(*data[code]) + ",";
            }
            else {
                res.clear();
                return ErrorType::INVALID_KEY;
            }
            res.pop_back();
            return ErrorType::NO_ERROR;
        }
    }
    else
        return ErrorType::UNSUPPORTED_TYPE;

    return ErrorType::UNSUPPORTED_TYPE;
}

void Mediator::ReleaseInstance()
{
    if (Mediator::m_instance != nullptr)
    {
        delete Mediator::m_instance;
        Mediator::m_instance = nullptr;
    }
}
Mediator::ErrorType Mediator::set(std::string &val, std::string key, std::string type, int fileds)
{
    if (type == "number")
    {
        if (data.find(key) == data.end())
            return ErrorType::INVALID_KEY;
        *data[key] = std::stod(val);
        return ErrorType::NO_ERROR;
    }
    else if (type == "vector")
    {
        for (int i = 0; i < fileds; i++)
        {
            std::string code = util::GetCode(key, i);
            if (data.find(code) == data.end())
            {
                return ErrorType::INVALID_KEY;
            }
        }
        auto vec = util::ParseVector(val);
        for (int i = 0; i < fileds; i++)
        {
            std::string code = util::GetCode(key, i);
            *data[code] = vec[i];
        }
        return ErrorType::NO_ERROR;
    }
    else
        return ErrorType::UNSUPPORTED_TYPE;
    return ErrorType::UNSUPPORTED_TYPE;
}

std::string Mediator::ErrorTypeStr(ErrorType type) {
    switch (type)
    {
    case ErrorType::NO_ERROR:
        return "No Error";
    case ErrorType::INVALID_KEY:
        return "Invalid Key";
    case ErrorType::UNSUPPORTED_TYPE:
        return "Unsupported Type";
    default:
        return "Unknown Error";
    }
}
Mediator::Mediator() {

}

/*
 * @Author: Amadeus
 * @Date: 2024-04-07 18:01:03
 * @LastEditors: Amadeus
 * @LastEditTime: 2024-04-11 19:34:21
 * @FilePath: /Satellite/src/General/APIHandler.cc
 * @Description: 
 */
#include"APIHandler.hh"
#include "httplib.h"
static Handler* m = Handler::GetInstance();
Handler::DeleteHelper Handler::helper;

Handler *Handler::GetInstance()
{
    if (m_instance == NULL)
        m_instance = new Handler;
    return m_instance;
}

void Handler::add(std::string key, double *value)
{
    std::cout << "Handler: " << key << " is " << value << std::endl;
    if (value == nullptr)
    {
        exit(0);
    }
    data[key] = value;
}
void Handler::ReleaseInstance()
{
    if (Handler::m_instance != nullptr)
    {
        delete Handler::m_instance;
        Handler::m_instance = nullptr;
    }
}

Handler::Handler() {

}

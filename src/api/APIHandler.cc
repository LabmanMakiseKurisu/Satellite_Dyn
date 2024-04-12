#include"APIHandler.hh"

void APIHandler::SetHeader(httplib::Response &res)
{
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization");
}

void APIHandler::handleOptions(const httplib::Request &req, httplib::Response &res) {
    SetHeader(res);
    res.set_header("Access-Control-Max-Age", "3600"); // 可选，告诉浏览器可以缓存预检请求多久（单位：秒）
    res.status = 200; // OK
}

//example http://localhost:8080/Data/SIMXXXX?type=matrix&row=10&col=10
void APIHandler::handleGet(const httplib::Request &req, httplib::Response &res)
{
    // 添加CORS头部
    SetHeader(res);

    // 从URL路径或查询参数中提取key（这取决于你如何设计URL）
    std::string key = req.matches[1]; // 假设key是通过路径参数传递的

    // 从查询参数中提取type、row和col
    std::string type = "";
    int row = 1;
    int col = 1;

    // 检查并获取type
    if (req.has_param("type"))
    {
        type = req.get_param_value("type");
    }

    // 检查并获取row
    if (req.has_param("row"))
    {
        row = std::stoi(req.get_param_value("row")); // 转换字符串为整数
    }

    // 检查并获取col
    if (req.has_param("col"))
    {
        col = std::stoi(req.get_param_value("col")); // 同上
    }

    // 使用Mediator获取数据
    std::string responseData;
    auto error = Mediator::GetInstance()->get(responseData, key, type, row);

    if (error == Mediator::ErrorType::NO_ERROR)
    {
        // 手动构造JSON响应体
        json responseJson;
        responseJson["type"] = type;
        if (row > 0)
            responseJson["row"] = row;
        if (col > 0)
            responseJson["col"] = col;
        responseJson["data"] = responseData;
        res.set_content(responseJson.dump(), "application/json");
    }
    else
    {
        // 错误处理
        res.status = 400; // Bad Request
        json responseJson;
        responseJson["error"] = Mediator::ErrorTypeStr(error);
        res.set_content(responseJson.dump(), "application/json");
    }
}

//example http://localhost:8080/Data/SIMXXXX
//body 
/*{
  "type": {
    "description": "Data type",
    "enum": ["number", "string", "vector", "matrix"]
  },
  "row": {
    "description": "Number of rows (applicable only for 'vector' and 'matrix' types)",
    "type": "integer",
    "required": false
  },
  "col": {
    "description": "Number of columns (applicable only for 'vector' and 'matrix' types)",
    "type": "integer",
    "required": false
  },
  "data": {
    "description": "Actual data content",
    "type": "object",
    "properties": {},
    "additionalProperties": true
  }
} */
void APIHandler::handlePut(const httplib::Request &req, httplib::Response &res)
{
    std::string key = req.matches[1]; // 假设key是通过路径参数传递的
    // 添加CORS头部
    SetHeader(res);

    // 解析请求体为JSON
    auto j = json::parse(req.body);
    auto type = j["type"].get<std::string>();
    auto data = j["data"].dump(); // 将数据作为字符串处理，根据实际情况可能需要转换
    int row = j["row"].get<int>();
    std::string responseMsg;
    auto error = Mediator::GetInstance()->set(data, key, type, row);

    if (error == Mediator::ErrorType::NO_ERROR)
    {
        json responseJson;
        responseJson["message"] = "Data updated successfully";
        res.set_content(responseJson.dump(), "application/json");
    }
    else
    {
        // 根据错误类型设置不同的响应
        res.status = 400; // Bad Request
        json responseJson;
        responseJson["error"] = Mediator::ErrorTypeStr(error);
        res.set_content(responseJson.dump(), "application/json");
    }
}
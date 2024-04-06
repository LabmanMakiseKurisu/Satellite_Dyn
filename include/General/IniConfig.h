#pragma once
#include <string>
#include <map>
#include"BaseMath.h"
//*.ini
class CIniConfig
{
public:
	CIniConfig()
	{
	}
	~CIniConfig()
	{
	}
	CIniConfig(const std::string& filename)
	{
		ReadConfig(filename);
	}
	bool ReadConfig(const std::string& filename);
	std::string ReadString(const char* section, const char* item, const char* default_value = "");
	int ReadInt(const char* section, const char* item, const int& default_value = 0);
	int64_t ReadInt64_t(const char* section, const char* item, const int& default_value = 0);
	double ReadDouble(const char* section, const char* item, const int& default_value = 0);
	float ReadFloat(const char* section, const char* item, const float& default_value = 0.0);
	Eigen::MatrixXd ReadMatrix(const char* section, const char* item);
	Eigen::VectorXd ReadVector(const char* section, const char* item);
private:
	bool IsSpace(char c);
	bool IsCommentChar(char c);
	void Trim(std::string& str);
	bool AnalyseLine(const std::string& line, std::string& section, std::string& key, std::string& value);

private:
	//std::map<std::string, std::string> settings_;
	std::map<std::string, std::map<std::string, std::string> >settings_;
};


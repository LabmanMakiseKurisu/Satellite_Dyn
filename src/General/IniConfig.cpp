#include "IniConfig.h"
#include <fstream>
#include <stdlib.h>

bool CIniConfig::IsSpace(char c)
{
	if (' ' == c || '\t' == c)
		return true;
	return false;
}

bool CIniConfig::IsCommentChar(char c)
{
	switch (c) {
	case '#':
		return true;
	default:
		return false;
	}
}

void CIniConfig::Trim(std::string& str)
{
	if (str.empty())
	{
		return;
	}
	int i, start_pos, end_pos;
	for (i = 0; i < str.size(); ++i) {
		if (!IsSpace(str[i])) {
			break;
		}
	}
	if (i == str.size())
	{
		str = "";
		return;
	}
	start_pos = i;
	for (i = str.size() - 1; i >= 0; --i) {
		if (!IsSpace(str[i])) {
			break;
		}
	}
	end_pos = i;
	str = str.substr(start_pos, end_pos - start_pos + 1);
}

bool CIniConfig::AnalyseLine(const std::string& line, std::string& section, std::string& key, std::string& value)
{
	if (line.empty())
		return false;
	int start_pos = 0, end_pos = line.size() - 1, pos, s_startpos, s_endpos;
	if ((pos = line.find("#")) != -1)
	{
		if (0 == pos)
		{
			return false;
		}
		end_pos = pos - 1;
	}
	if (((s_startpos = line.find("[")) != -1) && ((s_endpos = line.find("]"))) != -1)
	{
		section = line.substr(s_startpos + 1, s_endpos - 1);
		return true;
	}
	std::string new_line = line.substr(start_pos, start_pos + 1 - end_pos);
	if ((pos = new_line.find('=')) == -1)
		return false;
	key = new_line.substr(0, pos);
	value = new_line.substr(pos + 1, end_pos + 1 - (pos + 1));
	Trim(key);
	if (key.empty()) {
		return false;
	}
	Trim(value);
	if ((pos = value.find("\r")) > 0)
	{
		value.replace(pos, 1, "");
	}
	if ((pos = value.find("\n")) > 0)
	{
		value.replace(pos, 1, "");
	}
	return true;
}

bool CIniConfig::ReadConfig(const std::string& filename)
{
	settings_.clear();
	std::ifstream infile(filename.c_str());//����Ĭ�ϵ���open,���Կ��Բ�����open
	//std::ifstream infile;
	//infile.open(filename.c_str());
	//bool ret = infile.is_open()
	if (!infile) {
		return false;
	}
	std::string line, key, value, section;
	std::map<std::string, std::string> k_v;
	std::map<std::string, std::map<std::string, std::string> >::iterator it;
	while (getline(infile, line))
	{
		if (AnalyseLine(line, section, key, value))
		{
			it = settings_.find(section);
			if (it != settings_.end())
			{
				k_v[key] = value;
				it->second = k_v;
			}
			else
			{
				k_v.clear();
				settings_.insert(std::make_pair(section, k_v));
			}
		}
		key.clear();
		value.clear();
	}
	infile.close();
	return true;
}

std::string CIniConfig::ReadString(const char* section, const char* item, const char* default_value)
{
	std::string tmp_s(section);
	std::string tmp_i(item);
	std::string def(default_value);
	std::map<std::string, std::string> k_v;
	std::map<std::string, std::string>::iterator it_item;
	std::map<std::string, std::map<std::string, std::string> >::iterator it;
	it = settings_.find(tmp_s);
	if (it == settings_.end())
	{
		return def;
	}
	k_v = it->second;
	it_item = k_v.find(tmp_i);
	if (it_item == k_v.end())
	{
		return def;
	}
	return it_item->second;
}

int CIniConfig::ReadInt(const char* section, const char* item, const int& default_value)
{
	std::string tmp_s(section);
	std::string tmp_i(item);
	std::map<std::string, std::string> k_v;
	std::map<std::string, std::string>::iterator it_item;
	std::map<std::string, std::map<std::string, std::string> >::iterator it;
	it = settings_.find(tmp_s);
	if (it == settings_.end())
	{
		return default_value;
	}
	k_v = it->second;
	it_item = k_v.find(tmp_i);
	if (it_item == k_v.end())
	{
		return default_value;
	}
	return atoi(it_item->second.c_str());
}

int64_t CIniConfig::ReadInt64_t(const char* section, const char* item, const int& default_value )
{
	std::string tmp_s(section);
	std::string tmp_i(item);
	std::map<std::string, std::string> k_v;
	std::map<std::string, std::string>::iterator it_item;
	std::map<std::string, std::map<std::string, std::string> >::iterator it;
	it = settings_.find(tmp_s);
	if (it == settings_.end())
	{
		return default_value;
	}
	k_v = it->second;
	it_item = k_v.find(tmp_i);
	if (it_item == k_v.end())
	{
		return default_value;
	}
	return std::atoi(it_item->second.c_str());
}

double CIniConfig::ReadDouble(const char* section, const char* item, const int& default_value)
{
	std::string tmp_s(section);
	std::string tmp_i(item);
	std::map<std::string, std::string> k_v;
	std::map<std::string, std::string>::iterator it_item;
	std::map<std::string, std::map<std::string, std::string> >::iterator it;
	it = settings_.find(tmp_s);
	if (it == settings_.end())
	{
		return default_value;
	}
	k_v = it->second;
	it_item = k_v.find(tmp_i);
	if (it_item == k_v.end())
	{
		return default_value;
	}
	return atof(it_item->second.c_str());
}

float CIniConfig::ReadFloat(const char* section, const char* item, const float& default_value)
{
	std::string tmp_s(section);
	std::string tmp_i(item);
	std::map<std::string, std::string> k_v;
	std::map<std::string, std::string>::iterator it_item;
	std::map<std::string, std::map<std::string, std::string> >::iterator it;
	it = settings_.find(tmp_s);
	if (it == settings_.end())
	{
		return default_value;
	}
	k_v = it->second;
	it_item = k_v.find(tmp_i);
	if (it_item == k_v.end())
	{
		return default_value;
	}
	return atof(it_item->second.c_str());
}

Eigen::MatrixXd CIniConfig::ReadMatrix(const char* section, const char* item)
{
	std::string matrixString = ReadString(section, item);
	std::vector<double> matrixValues;
	std::vector<int> colCounts;
	int colCount = 0;

	std::stringstream ss(matrixString);
	std::string rowString;

	// ����ÿһ��
	while (std::getline(ss, rowString, ';')) {
		std::stringstream rowStream(rowString);
		std::string cell;

		// ����ÿ��Ԫ��
		while (std::getline(rowStream, cell, ',')) {
			matrixValues.push_back(std::stod(cell));
			colCount++;
		}

		colCounts.push_back(colCount);
		colCount = 0;
	}

	// ���ÿ�е������Ƿ���ͬ
	for (size_t i = 1; i < colCounts.size(); ++i) {
		if (colCounts[i] != colCounts[0]) {
			throw std::runtime_error("Matrix rows have varying number of columns");
		}
	}

	// ת��ΪEigen����
	int rows = colCounts.size();
	int cols = colCounts.empty() ? 0 : colCounts[0];
	Eigen::MatrixXd matrix(rows, cols);

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			matrix(i, j) = matrixValues[i * cols + j];
		}
	}

	return matrix;
}

Eigen::VectorXd CIniConfig::ReadVector(const char* section, const char* item)
{
	std::string vectorString = ReadString(section, item);
	std::vector<double> vectorValues;

	std::stringstream ss(vectorString);
	std::string cell;

	// ����ÿ��Ԫ��
	while (std::getline(ss, cell, ',')) {
		vectorValues.push_back(std::stod(cell));
	}

	// ת��ΪEigen����
	int size = vectorValues.size();
	Eigen::VectorXd vector(size);

	for (int i = 0; i < size; ++i) {
		vector[i] = vectorValues[i];
	}

	return vector;
}



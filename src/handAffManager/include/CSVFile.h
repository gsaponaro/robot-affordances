/*
 * Copyright: (C) 2015 Vladimir Shestakov
 * http://stackoverflow.com/a/28840805/1638888
 * https://github.com/rudolfovich
 *
 * modified by Giovanni Saponaro <gsaponaro@isr.tecnico.ulisboa.pt>
 *
 */

#ifndef CSVFILE_H
#define CSVFILE_H
#pragma once

#include <string>
#include <iostream>
#include <fstream>

class csvfile;

inline static csvfile& endrow(csvfile& file);
inline static csvfile& flush(csvfile& file);

class csvfile
{
	std::ofstream fs_;
	const std::string separator_;
public:
	// Giovanni
	csvfile()
		: fs_()
		, separator_(";")
	{
		fs_.exceptions(std::ios::failbit | std::ios::badbit);
	}
	void setFilename(const std::string filename)
	{
		fs_.exceptions(std::ios::failbit | std::ios::badbit);
		fs_.open(filename.c_str());
	}

	csvfile(const std::string filename, const std::string separator = ";")
		: fs_()
		, separator_(separator)
	{
		fs_.exceptions(std::ios::failbit | std::ios::badbit);
		fs_.open(filename.c_str()); // Giovanni http://stackoverflow.com/a/6323624/1638888
	}

	~csvfile()
	{
		flush();
		fs_.close();
	}

	void flush()
	{
		fs_.flush();
	}

	void endrow()
	{
		fs_ << std::endl;
	}

	csvfile& operator << ( csvfile& (* val)(csvfile&))
	{
		return val(*this);
	}

	csvfile& operator << (const char * val)
	{
		fs_ << '"' << val << '"' << separator_;
		return *this;
	}

	csvfile& operator << (const std::string & val)
	{
		fs_ << '"' << val << '"' << separator_;
		return *this;
	}

	template<typename T>
	csvfile& operator << (const T& val)
	{
		fs_ << val << separator_;
		return *this;
	}
};


inline static csvfile& endrow(csvfile& file)
{
	file.endrow();
	return file;
}

inline static csvfile& flush(csvfile& file)
{
	file.flush();
	return file;
}

#endif // CSVFILE_H

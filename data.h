#pragma once
#include <list>
#include <string>
#include <memory>
#include "SystemParser.h"

namespace data 
{
	using _system = std::list<std::unique_ptr<sysparser::eqNode>>;

	struct inputdata
	{
		sysparser::eqNode func; // Функция цели
		_system system;         // Система
	};

	void fileReader(inputdata&, const std::string& filename, sysparser::Mode mode = sysparser::Mode::DEFAULT);
	void clearUserData(inputdata&);

} // namespace user_data

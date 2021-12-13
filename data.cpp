#include <iostream>
#include <fstream>
#include "data.h"

namespace data 
{
	void fileReader(inputdata& id, const std::string& filename, sysparser::Mode mode)
	{
		std::string buf;
		std::string err;
		size_t err_pos;

		std::ifstream in(filename);

		if (!in.is_open())
		{
			std::cerr << "CANNOT OPEN THE FILE" << std::endl;
			exit(-1);
		}

		getline(in, buf);
		err = sysparser::parse(buf, id.func, err_pos, sysparser::eqtype::func, mode);
		if (!err.empty())
		{
			std::cerr << err_pos << ": " << err << "BAD FUNCTION" << std::endl;
			exit(-2);
		}

		std::cout << "GOOD FUNC [" << sysparser::recreate(id.func, sysparser::eqtype::func) << "]" << std::endl;

		err.clear();
		while (getline(in, buf))
		{
			if (buf.compare("") == 0) 
			{
				if (id.system.size() < 2) 
				{
					std::cerr << "to low limits" << std::endl;
					exit(-3);
				}
				else
					break;
			}

			id.system.push_back(std::make_unique<sysparser::eqNode>());
			err = sysparser::parse(buf, *id.system.back(), err_pos, sysparser::eqtype::ineq, mode);

			if (!err.empty())
			{
				std::cerr << "ERROR: " << err << " in line: " << err_pos << std::endl;
				exit(-4);
			}
			else 
			{
				std::cout << "LIMITS CONFIRM: " << sysparser::recreate(*id.system.back(), sysparser::eqtype::ineq) << std::endl;
			}
		}

		in.close();
	}

	// Memory cleaner
	void clearUserData(inputdata& dt)
	{
		for (auto& item : dt.system)
		{
			item->vars.clear();
		}

		dt.system.clear();
		dt.func.vars.clear();
	}

} // namespace data

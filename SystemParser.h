#pragma once
#include <list>
#include <string>
#include <string_view>

namespace sysparser
{
	enum class Comparison
	{
		Less, 
		LessOrEqual,
		Greater, 
		GreaterOrEqual,
		Equal
	};

	enum class Task
	{
		MIN, 
		MAX
	};

	enum class arthsign
	{
		minus, plus, mul, div
	};

	enum class eqtype
	{
		ineq, func
	};

	enum class Mode
	{
		DEFAULT,
		WITHPARAMS
	};

	struct Node
	{
		double number;
		size_t index;
	};
	
	using _vars = std::list<Node>;

	struct eqNode
	{
		_vars vars;
		Comparison sign;
		Task task;
		double rval;
	};

	// Парсим систему (два режима)
	std::string parse(std::string_view, eqNode&, size_t&, eqtype, Mode mode = Mode::DEFAULT);
	// Функция восстанавливает систему линейных уравнений с целевой функцией
	std::string recreate(const eqNode&, const eqtype);
}



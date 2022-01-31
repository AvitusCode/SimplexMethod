#include <iostream>
#include <sstream>
#include <iomanip>

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

#include "SystemParser.h"

#define isnum(x) (x >= '0' && x <= '9')
#define iseqsign(x) (x == '<' || x == '=' || x == '>')
#define isarithsign(x) (x == '+' || x == '-')
#define isparam(x) ((x >= 'a' && x <= 'z') || (x >= 'A' && x <= 'Z'))

namespace sysparser 
{

    // тип данных пришедший посредством считывания строки (позволяет переключать состояние автомата)
	enum class _type
	{
		none,
		num,
		arith_sign,
		eq_sign, 
		x,
		end, 
		space,
		min_max,
		approach
	};

	// Состояния конечного автомата
	enum class POS
	{
		start,       // Начальная позиция 
		start_as,    // ждем число
		a,           // ждем параметр
		at_x,        // индекс
		b,           // после индекса
		c,           // конец
		es,          // число или знак выражения
		as,          // число или параметр
		es_c,        // число или знак выраж.
		at_approach, // тип задачи
		at_lim       // окончание
	};

	enum class RC
	{
		NONE,
		FAIL,
		WIN
	};

	struct Token 
	{
		double num;
		_type type;
		Comparison eqsign;
		arthsign arithsign;
		Task task;
	};
	
	// Дефолтный парсер для значений, содержащих числа и знаки сравнения (работает)
	std::string parseDef(std::string&& s, eqNode& eq, eqtype _eqtype)
	{
		Node type;
		size_t INDEX = 1;
		Token token;
		std::string err = "";
		bool minus_flag = true;
		RC result = RC::NONE;

		std::istringstream inputstrm(std::move(s));
		eq.vars.clear();
		
		while (!inputstrm.eof() && inputstrm.peek() != '\n' && inputstrm.peek() != '\0')
		{
			inputstrm >> std::ws;

			if (isnum(inputstrm.peek()))
			{
				inputstrm >> type.number;
				
				type.number *= 2.0 * minus_flag - 1.0;
				type.index = INDEX++;
				eq.vars.push_back(type);
				minus_flag = true;
				
				continue;
			}
			
			char c = static_cast<char>(inputstrm.get());

			if (c == '-' && _eqtype == eqtype::func)
			{
				if (inputstrm.get() == '>')
				{
					inputstrm >> std::ws;

					std::string tsk;
					inputstrm >> tsk;

					if (tsk == "MAX" || tsk == "max")
						token.task = Task::MAX;
					else if (tsk == "MIN" || tsk == "min")
						token.task = Task::MIN;
					else
					{
						err = "ERROR: incorrect task initilizer";
						result = RC::FAIL;
					}

					eq.task = token.task;
					eq.rval = 0.0;
					break;
				}
			}
			else if (c == '-' && isnum(inputstrm.peek()))
			{
				minus_flag = false;
				continue;
			}
			else if (iseqsign(c))
			{
				if (c == '<')
				{
					token.eqsign = Comparison::Less;
					if (inputstrm.get() == '=')
						token.eqsign = Comparison::LessOrEqual;
				}
				else if (c == '>')
				{
					token.eqsign = Comparison::Greater;
					if (inputstrm.get() == '=')
						token.eqsign = Comparison::GreaterOrEqual;
				}
				else if (c == '=')
				{
					token.eqsign = Comparison::Equal;
				}
				else
				{
					err = "ERROR: eqsign expected";
					result = RC::FAIL;
				}

				inputstrm >> std::ws;
				inputstrm >> eq.rval;

				eq.sign = token.eqsign;
				break;
			}
			
			if (result == RC::FAIL)
				break;
		}

		return err;
	}

	// Отлаженный и работающий парсер для выражений с параметрами
	std::string parseWithParams(std::string_view sv, eqNode& eq, size_t& err_pos, eqtype _eqtype)
	{
		if (sv.empty()){
			return "ERROR: empty string!";
		}
		
		Node type;
		
		size_t i = 0;
		char param = 0;
		Token tok; // Тип токена
		std::string err = "";  // Тип ошибки
		
		POS machinePos = POS::start; // Отвечает за позицию в конечном автомате
		RC result = RC::NONE;        // returne code

		eq.vars.clear();

		while (true) 
		{
			err_pos = i + 1;
			tok.type = _type::none;

			if (i >= sv.size())
			{
				result = RC::WIN;
				break;
			}

			while (sv[i] == ' ' || sv[i] == '\t') 
			{
				tok.type = _type::space;
				i++;
			}

			if (sv[i] == '\0' || sv[i] == '\n') 
			{
				tok.type = _type::end;
				i++;
			}

			if (tok.type == _type::none && isnum(sv[i])) 
			{
				tok.type = _type::num;
				
				if (i == 0 && sv[i] == '0'){
					tok.num = 0;
					i++;
				}
				else if (sv[i] == '0' && !isnum(sv[i - 1]) && isparam(sv[i + 1])){
					tok.num = 0;
					i++;
				}
				else
					for (tok.num = atof(&sv[i]); i < sv.size(); i++)
					{
						if (!(isnum(sv[i]) || sv[i] == '.'))
							break;
					}
			}

			// Определяем тип задачи
			if (tok.type == _type::none) 
			{
				if (strncmp(&sv[i], "min", 3) == 0) 
				{
					tok.type = _type::min_max;
					tok.task = Task::MIN;
					i += 3;

				}
				else if (strncmp(&sv[i], "max", 3) == 0) 
				{
					tok.type = _type::min_max;
					tok.task = Task::MAX;
					i += 3;
				}
			}

			// Перед нами параметр
			if (tok.type == _type::none) 
			{
				if (param == 0) 
				{
					if (isparam(sv[i]))
					{
						param = sv[i];
						tok.type = _type::x;
						i++;
					}
				}
				else 
				{
					if (param == sv[i]) 
					{
						tok.type = _type::x;
						i++;
					}
				}
			}

			//Проверяем синтаксическую корректность
			if (tok.type == _type::none && (strncmp(&sv[i], "->", 2) == 0 || strncmp(&sv[i], "=>", 2) == 0)) 
			{
				tok.type = _type::approach;
				i += 2;
			}

			// определяем знак неравенства
			if (tok.type == _type::none && iseqsign(sv[i])) 
			{
				tok.type = _type::eq_sign;
				
				switch (sv[i]) 
				{
				case '<':
					tok.eqsign = Comparison::Less;
					i++;

					if (sv[i] == '=') 
					{
						tok.eqsign = Comparison::LessOrEqual;
						i++;
					}
					break;
				case '>':
					tok.eqsign = Comparison::Greater;
					i++;
					if (sv[i] == '=') 
					{
						tok.eqsign = Comparison::GreaterOrEqual;
						i++;
					}
					break;
				case '=':
					tok.eqsign = Comparison::Equal;
					i++;
					break;
				}
			}
			

			// пришла операция
			if (tok.type == _type::none && isarithsign(sv[i])) 
			{
				tok.type = _type::arith_sign;
				switch (sv[i]) 
				{
				case '+':
					tok.arithsign = arthsign::plus;
					i++;
					break;
				case '-':
					tok.arithsign = arthsign::minus;
					i++;
					break;
				}
			}

			// Определяем состояние конечного автомата и производим соответствующую реакцию
			switch (machinePos) 
			{
			case POS::start: // space, arith_sign, num, x
				if (tok.type == _type::space){
					break;
				}
				
				if (tok.type == _type::num)
				{
					type.number = tok.num;
					machinePos = POS::a;
					break;
				}
				
				if (tok.type == _type::arith_sign) 
				{
					type.number = tok.arithsign == arthsign::minus ? -1.0 : 1.0;
					machinePos = POS::start_as;
					break;
				}
				
				if (tok.type == _type::x) 
				{
					type.number = 1;
					machinePos = POS::at_x;
					break;
				}

				err = "ERROR: space, number, sign or x expected";
				result = RC::FAIL;
				break;
			case POS::start_as: // num
				if (tok.type == _type::num) 
				{
					machinePos = POS::a;
					type.number *= tok.num;
					break;
				}
				
				if (tok.type == _type::x)
				{
					machinePos = POS::at_x;
					break;
				}

				err = "ERROR: number or x expected";
				result = RC::FAIL;
				break;
			case POS::a: // x
				if (tok.type == _type::space) {
					break;
				}

				if (tok.type == _type::x) 
				{
					machinePos = POS::at_x;
					break;
				}

				err = "ERROR: x expected";
				result = RC::FAIL;
				break;
			case POS::at_x:
				if (tok.type == _type::num) 
				{
					machinePos = POS::b;

					/*Проверяем индек на натуральность*/
					double remains = fabs(tok.num - static_cast<int>(tok.num));
					
					if (remains != 0 && remains > 0.00001) 
					{
						err = "ERROR: index of x has to be ceil!";
						result = RC::FAIL;
						break;
					}
					type.index = static_cast<size_t>(tok.num);

					if (eq.vars.empty()) {
						eq.vars.push_back(type);
					}
					
					else 
					{
					// Если такой индекс уже имелся, то объединяем числа
						bool found_match = false;
						for (auto it = eq.vars.begin(); it != eq.vars.end(); it++) 
						{
							if (it->index == type.index) 
							{
								it->number += type.number;
								found_match = true;
								break;
							}
						}
						if (!found_match) 
						{
							eq.vars.push_back(type);
						}
						
						found_match = false;
					}
					break;
				}

				err = "ERROR: x is neccessary";
				result = RC::FAIL;
				break;
			case POS::b: // space, arith_sign, eq_sign
				if (tok.type == _type::space)
				{
					break;
				}
				
				if (tok.type == _type::arith_sign)
				{
					machinePos = POS::as;
					type.number = tok.arithsign == arthsign::minus ? -1 : 1;
					break;
				}
				
				if (_eqtype == eqtype::func && tok.type == _type::approach) 
				{
					machinePos = POS::at_approach;
					break;
				}
				if (_eqtype == eqtype::ineq && tok.type == _type::eq_sign) 
				{
					machinePos = POS::es;
					eq.rval = 1;
					if (tok.eqsign == Comparison::Less || tok.eqsign == Comparison::Greater) 
					{
						err = "ERROR: < or >";
						result = RC::FAIL;
						break;
					}
					eq.sign = tok.eqsign;
					break;
				}

				if (_eqtype == eqtype::ineq || _eqtype == eqtype::func)
					err = "ERROR";
				result = RC::FAIL;

				break;
			// 	 После стрелочки у функции цели должен быть токен min or max
			case POS::at_approach:
				if (tok.type == _type::space){
					break;
				}
				
				if (tok.type == _type::min_max)
				{
					machinePos = POS::at_lim;
					eq.task = tok.task;
					break;
				}

				err = "ERROR: max or min when was -> expected";
				result = RC::FAIL;
				break;
			case POS::as: // space, num, x
				if (tok.type == _type::space) {
					break;
				}
				
				if (tok.type == _type::num) 
				{
					machinePos = POS::a;
					type.number *= tok.num;
					break;
				}
				if (tok.type == _type::x) 
				{
					machinePos = POS::at_x;
					break;
				}

				err = "ERROR: space or x expected";
				result = RC::FAIL;
				break;
			case POS::es: // space, num, arith_sign
				if (tok.type == _type::space) {
					break;
				}
				
				if (tok.type == _type::arith_sign) 
				{
					machinePos = POS::es_c;
					if (tok.arithsign == arthsign::minus)
						eq.rval = -1;
					
					break;
				}
				
				if (tok.type == _type::num) 
				{
					machinePos = POS::c;
					eq.rval = tok.num;
					break;
				}

				err = "ERROR: space or x expected";
				result = RC::FAIL;
				break;
			case POS::es_c:
				if (tok.type == _type::num) 
				{
					machinePos = POS::c;
					eq.rval *= tok.num;
					break;
				}

				err = "ERROR: number expected";
				result = RC::FAIL;
				break;
			case POS::c:
				if (tok.type == _type::space) {
					break;
				}
				if (tok.type == _type::end) 
				{
					result = RC::WIN;
					break;
				}

				err = "ERROR: space or end expected";
				result = RC::FAIL;
				break;
			case POS::at_lim:
				if (tok.type == _type::space) {
					break;
				}
				
				if (tok.type == _type::end) 
				{
					result = RC::WIN;
					break;
				}

				err = "ERROR: space or end expected";
				result = RC::FAIL;
				break;
			}

			if (result == RC::FAIL || result == RC::WIN)
				break;
		}

		if (result == RC::WIN)
		{
			//Сортируем значения по индексам
			eq.vars.sort([](const Node& a, const Node& b) {
				return a.index < b.index;
				});
		}

		return err;
	}

	std::string parse(std::string_view sv, eqNode& eq, size_t& err_pos, eqtype eqtype, Mode mode)
	{
		if (mode == Mode::DEFAULT)
			return parseDef(std::string(sv), eq, eqtype);
		else if (mode == Mode::WITHPARAMS)
			return parseWithParams(sv, eq, err_pos, eqtype);

		return "ERROR: undefined mode token";
	}

	std::string recreate(const eqNode& eq, const eqtype et)
	{
		std::ostringstream ss;

		for (auto it = eq.vars.cbegin(); it != eq.vars.cend(); it++) 
		{
			if (it == eq.vars.cbegin()) 
			{
				if (it->number < 0 && fabs(it->number) == 1)
					ss << "-";
			}
			else
			{
				if (it->number > 0)
					ss << "+ ";
				else
					ss << "- ";
			}
			
			if (fabs(it->number) != 1)
				ss << fabs(it->number);
			
			ss << "x" << it->index << " ";
		}
		
		if (et == eqtype::func) 
		{
			ss << "-> ";
			ss << (eq.task == Task::MAX ? "max" : "min");
		}
		
		else 
		{ 
			switch (eq.sign) 
			{
			case Comparison::Equal:
				ss << "= ";
				break;
			case Comparison::Less:
				ss << "< ";
				break;
			case Comparison::Greater:
				ss << "> ";
				break;
			case Comparison::GreaterOrEqual:
				ss << "=> ";
				break;
			case Comparison::LessOrEqual:
				ss << "<= ";
				break;
			default:
				break;
			}
			
			ss << eq.rval;
		}

		return ss.str();
	}

} // namespace systemparser

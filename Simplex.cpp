#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "Simplex.h"

//#define DEBUG

#ifdef DEBUG
#include <algorithm>
#include <iterator>
#endif // DEBUG

// Выражаем переменные искуственного базиса через основные переменные и исправляем функцию цели
void Simplex::fixTargetFunction(const std::vector<size_t>& vecY)
{
	// TODO: поразмыслить над введением константы (Испробовано, все работает, но с исключениями)
	M *= 100.0;
	const bool minus = wayOfTargetFunction == sysparser::Task::MAX ? false : true;

    #ifdef DEBUG
	std::copy(new_plane->indexString.cbegin(), new_plane->indexString.cend(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;
    #endif // DEBUG

	for (size_t i = 0; i < vecY.size(); i++)
	{
		if (vecY[i])
		{
			for (int j = static_cast<int>(new_plane->basisVars.At(0, i)) - 2; j >= 0; j--){
				new_plane->indexString[j] += (2.0 * minus - 1.0) * (new_plane->varsFactors.At(i, j) * M);
			}
			new_plane->targetFunction += (2.0 * minus - 1.0) * (new_plane->basisVars.At(1, i));
		}
	}

    #ifdef DEBUG
	std::copy(new_plane->indexString.cbegin(), new_plane->indexString.cend(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;
    #endif // DEBUG

	new_plane->targetFunction *= M;
}

void Simplex::generate(const data::inputdata& ud)
{
	std::shared_ptr<Plan> p;
	
	res = result::bad_solution;
	wayOfTargetFunction = ud.func.task;
	numOfSourceVars = ud.func.vars.size();
	numOfSourceRow = ud.system.size();
	old_plane = std::make_shared<Plan>(numOfSourceVars, numOfSourceRow);
	new_plane = p = std::make_shared<Plan>(numOfSourceVars, numOfSourceRow);

	for (const auto& edit : ud.func.vars){
		M = edit.number > M ? edit.number : M; // Подбираем константу M. Она точно должна быть больше всех значений координат функции цели
		p->indexString[edit.index - 1] = edit.number * (-1);
	}

	// Создадим вспомогательный массив для добавления еще одного базиса
	std::vector<size_t> vecY(numOfSourceRow, 0);
	
	size_t i = 0, cBasis = 0;
	for (const auto& item : ud.system)
	{
		for (const auto& edit : item->vars){
			p->varsFactors.At(i, edit.index - 1) = edit.number;
		}

		p->basisVars.At(1, i) = item->rval;

		
		switch (item->sign)
		{
		case sysparser::Comparison::LessOrEqual:
			p->varsFactors.At(i, numOfSourceVars + cBasis) = 1;
			vecY[i] = 0;
			cBasis++;
			break;
		case sysparser::Comparison::Less:
			p->varsFactors.At(i, numOfSourceVars + cBasis) = 1;
			vecY[i] = 0;
			cBasis++;
			break;
		case sysparser::Comparison::Equal:
			p->varsFactors.At(i, numOfSourceVars + cBasis) = 0;
			vecY[i] = 1;
			break;
		case sysparser::Comparison::GreaterOrEqual:
			p->varsFactors.At(i, numOfSourceVars + cBasis) = -1;
			vecY[i] = 1;
			cBasis++;
			break;
		case sysparser::Comparison::Greater:
			p->varsFactors.At(i, numOfSourceVars + cBasis) = -1;
			vecY[i] = 1;
			cBasis++;
			break;
		}
		
		i++;
	}
	
	// Теперь сделаем послдений проход для добавления дополнительного базиса
	size_t yBasis = cBasis;
	size_t xBasis = numOfSourceVars + 1;
	for (i = 0; i < vecY.size(); i++)
	{
		if (vecY[i])
		{
			if (p->varsFactors.GetNumColumns() <= numOfSourceVars + yBasis)
			{
				p->varsFactors.Resize(numOfSourceRow, numOfSourceVars + yBasis + 1);
				p->indexString.resize(numOfSourceVars + yBasis + 1);
				old_plane->varsFactors.Resize(numOfSourceRow, numOfSourceVars + yBasis + 1);
				old_plane->indexString.resize(numOfSourceVars + yBasis + 1);
			}

			p->varsFactors.At(i, numOfSourceVars + yBasis) = 1;
			p->basisVars.At(0, i) = numOfSourceVars + yBasis + 1;
			yBasis++;

			if (p->varsFactors.At(i, xBasis - 1) != 0.0) {
				xBasis++;
			}
		}
		else
		{
			p->basisVars.At(0, i) = xBasis++; 
		}
	}

	// Исправляем функцию цели для табличного метода посредством добавления штрафных функций (если нужно)
	fixTargetFunction(vecY);

	//для отладки
    #ifdef DEBUG
	std::cout << p->varsFactors << std::endl;
	std::cout << p->basisVars << std::endl;
    #endif

	setIndexOfLeavingColumn(p);
	setThColumn(p);
	setIndexOfLeavingRow(p);
	setAllowingMember(p);
}

void closeFile(std::ofstream& file)
{
	if (file.is_open())
		file.close();
}

result Simplex::run()
{
	static size_t i;
	result r;
	std::ofstream file(outFile);
	
	while (true) 
	{
		if (res != result::good_solution)
		{
			if (result::good_solution == (r = checkPlane(new_plane)))
			{
				dumpToTableTxt(new_plane, i, r, file);
				
				if (pm_ == printMode::PRINT){
				    displayResult(new_plane, i, r);
				}
				
				res = r;
				closeFile(file);
				return r;
			}
			else
			{
				if (!checkThColumn(new_plane, old_plane))
				{
					r = result::no_solution;
					dumpToTableTxt(new_plane, i, r, file);
					
					if (pm_ == printMode::PRINT){
					    displayResult(new_plane, i, r);
					}
					
					res = r;
					closeFile(file);
					return r;
				}
				
				dumpToTableTxt(new_plane, i, r, file);
			}
		}

		if (new_plane->allowingMember == 0 || std::fabs(new_plane->allowingMember) < ZERO)
		{
			closeFile(file);
			return result::no_solution;
		}

		i++; // Swap plans and starting new iteration
		old_plane.swap(new_plane);

		setTargetFunction(old_plane, new_plane);
		setBasisVars(old_plane, new_plane);
		setIndexString(old_plane, new_plane);
		setFactorsOfVars(old_plane, new_plane);

		
		setIndexOfLeavingColumn(new_plane);
		setThColumn(new_plane);
		setIndexOfLeavingRow(new_plane);
		setAllowingMember(new_plane);
	
		res = result::bad_solution;
	}
}

// Если после деления свободных членов на разрешающий элемент - чилса отрицательны (все) или ERROR, то решения нет
bool Simplex::checkThColumn(const std::shared_ptr<Plan>& p, const std::shared_ptr<Plan>& old) const
{
	for (const auto& item : p->thColumn)
	{
		if (item > 0 && item != std::numeric_limits<double>::max()){
			return true;
		}
	}

	return false;
}

result Simplex::checkPlane(const std::shared_ptr<Plan>& p) const
{
	static bool flag;

	for (const auto& item : p->indexString)
	{
		if (wayOfTargetFunction == sysparser::Task::MAX)
		{
			if (item < 0) 
				return result::bad_solution;
		}
		else 
		{
			if (!flag)
			{
				flag = true;
				return result::bad_solution;
			}

			if (item > 0) 
				return result::bad_solution;
		}
	}

	for (size_t i = 0; i < p->basisVars.GetNumColumns(); i++)
	{
		if (p->basisVars.At(1, i) < 0)
			return result::bad_solution;
	}

	return result::good_solution;
}

// Выставить индекс ведущего столбца (Для минимизации нужно выбрать Максимальный неотрицательный элемент, для задачи максимизации - Минимальный отрицательный
void Simplex::setIndexOfLeavingColumn(std::shared_ptr<Plan>& p) 
{
	const bool what_task = wayOfTargetFunction == sysparser::Task::MAX ? true : false;
	double mOfIndexString = what_task ? std::numeric_limits<double>::max() : std::numeric_limits<double>::min();
	p->indexOfLeavingColumn = 0;

	for (size_t i = 0; i < p->indexString.size(); i++) 
	{
		if ((what_task && (mOfIndexString > p->indexString[i]) && p->indexString[i] < 0) || mOfIndexString == 0)
		{
			mOfIndexString = p->indexString[i];
			p->indexOfLeavingColumn = i;
		}
		else if (!what_task && mOfIndexString < p->indexString[i] && p->indexString[i] > 0)
		{
			mOfIndexString = p->indexString[i];
			p->indexOfLeavingColumn = i;
		}
	}
}

// Выставить индекс ведущей строки
// TODO: Нужно учесть случай, когда в тета массиве имеется два одинаковых элемента, иначе мы не сможем найти решение
void Simplex::setIndexOfLeavingRow(std::shared_ptr<Plan>& p)
{
	double minOfThColumn = std::numeric_limits<double>::max();
	p->indexOfLeavingRow = 0;

	for (size_t i = 0; i < p->thColumn.size(); i++) 
	{
		if ((minOfThColumn > p->thColumn[i] || minOfThColumn < 0) && p->thColumn[i] >= 0)
		{
			minOfThColumn = p->thColumn[i];
			p->indexOfLeavingRow = i;
		}
	}
}

// Разрешающий элемент
void Simplex::setAllowingMember(std::shared_ptr<Plan>& p) {
	p->allowingMember = p->varsFactors.At(p->indexOfLeavingRow, p->indexOfLeavingColumn);
}

void Simplex::setBasisVars(const std::shared_ptr<Plan>& source, std::shared_ptr<Plan>& target)
{
	double A = source->basisVars.At(1, source->indexOfLeavingRow);

	for (size_t i = 0; i < source->basisVars.GetNumColumns(); i++)
	{
		if (i == source->indexOfLeavingRow) 
		{
			target->basisVars.At(0, source->indexOfLeavingRow) = source->indexOfLeavingColumn + 1;
			target->basisVars.At(1, i) = source->basisVars.At(1, i) / source->allowingMember;
		}
		else 
		{
			target->basisVars.At(0, i) = source->basisVars.At(0, i);
			double B = source->varsFactors.At(i, source->indexOfLeavingColumn);
			target->basisVars.At(1, i) = source->basisVars.At(1, i) - ((A * B) / source->allowingMember);
		}
	}
}

void Simplex::setFactorsOfVars(const std::shared_ptr<Plan>& source, std::shared_ptr<Plan>& target) 
{
	for (size_t i = 0; i < source->varsFactors.GetNumRows(); i++) 
	{
		for (size_t j = 0; j < source->varsFactors.GetNumColumns(); j++) 
		{
			if (i == source->indexOfLeavingRow) 
			{
				target->varsFactors.At(i, j) = source->varsFactors.At(i, j) / source->allowingMember;
			}
			else 
			{
				double A = source->varsFactors.At(source->indexOfLeavingRow, j);
				double B = source->varsFactors.At(i, source->indexOfLeavingColumn);
				target->varsFactors.At(i, j) = source->varsFactors.At(i, j) - ((A * B) / source->allowingMember);
			}
		}
	}
}

void Simplex::setIndexString(const std::shared_ptr<Plan>& source, std::shared_ptr<Plan>& target) 
{
	for (size_t i = 0; i < source->indexString.size(); i++) 
	{
		double A = source->varsFactors.At(source->indexOfLeavingRow, i);
		double B = source->indexString[source->indexOfLeavingColumn];
		target->indexString[i] = source->indexString[i] - ((A * B) / source->allowingMember);
	}
}

void Simplex::setTargetFunction(const std::shared_ptr<Plan>& source, std::shared_ptr<Plan>& target) 
{
	double A = source->basisVars.At(1, source->indexOfLeavingRow);
	double B = source->indexString[source->indexOfLeavingColumn];
	target->targetFunction = source->targetFunction - ((A * B) / source->allowingMember);
}

void Simplex::setThColumn(std::shared_ptr<Plan>& p) 
{
	for (size_t i = 0; i < p->thColumn.size(); i++)
	{
		if (std::fabs(p->varsFactors.At(i, p->indexOfLeavingColumn)) < ZERO){
			p->thColumn[i] = std::numeric_limits<double>::max(); // ОШАБКА! попытка деления на нуль!
		}
		else
		{
			// Данные проверки позволяют избежать зацикливания (большинства случаев). Также необходимо делить только в том случае, когда переменные 
			// вектора ограничений и разрещающего столбца одного знака
			if ((p->varsFactors.At(i, p->indexOfLeavingColumn) < 0 && p->basisVars.At(1, i) < 0) || 
				(p->varsFactors.At(i, p->indexOfLeavingColumn) > 0 && p->basisVars.At(1, i) > 0) ||
				(p->varsFactors.At(i, p->indexOfLeavingColumn) > 0 && std::fabs(p->basisVars.At(1, i)) < ZERO)){
			    p->thColumn[i] = p->basisVars.At(1, i) / p->varsFactors.At(i, p->indexOfLeavingColumn);
			}
			else {
			    p->thColumn[i] = std::numeric_limits<double>::max();
			}
		}
	}
}


// Print some information
void Simplex::displayResult(const std::shared_ptr<Plan>& p, size_t iteration, result r) const
{
	std::ostringstream ss;
	if (r == result::good_solution)
	{
		ss << "\nOptimal plan was goal with " << iteration << " iterations\n" << std::endl;
		
		for (size_t i = 0; i < p->basisVars.GetNumColumns(); i++)
			ss << "x" << p->basisVars.At(0, i) << " = " << p->basisVars.At(1, i) << std::endl;
		
		ss << "\nf(x) = " << p->targetFunction << "\n";
	}

	if (r == result::no_solution)
		ss << "Function does not limit." << std::endl;

	std::cout << ss.str() << std::flush;
}

void Simplex::dumpToTableTxt(const std::shared_ptr<Plan>& p, size_t iteration, result r, std::ofstream& file) const
{
	if (!file.is_open())
	{
		return;
	}
		
	size_t i, j;
	std::ostringstream buf;

	buf << "\n\n";
	buf << "Plan(" << iteration << "):\n\n";

	for (i = 0; i < p->basisVars.GetNumColumns(); i++)
	{
		buf << "x" << std::setw(4) << std::left << std::fixed << std::setprecision(2) << p->basisVars.At(0, i) << ":\t";
		buf << std::setw(5) << std::fixed << std::setprecision(2) << p->basisVars.At(1, i) << "\t";
		
		for (j = 0; j < p->varsFactors.GetNumColumns(); j++)
			buf << std::setw(7) << std::fixed << std::setprecision(2) << p->varsFactors.At(i, j) << "	";
		
		if (r == result::bad_solution)
		{
			if (p->thColumn[i] == std::numeric_limits<double>::max()){
				buf << '-';
			}
			else{
			    buf << p->thColumn[i];
			}
		}
		
		buf << "\n\n";
	}

	buf << "f(x):\t" << std::setw(5) << std::fixed << std::setprecision(2) << p->targetFunction << "\t";

	for (i = 0; i < p->indexString.size(); i++)
		buf << std::setw(7) << std::fixed << std::setprecision(2) << p->indexString[i] << "\t";

	buf << "\n\n";

	switch (r) 
	{
	case result::bad_solution:
		buf << "This plan need to be optimized.";
		break;
	case result::good_solution:
		buf << "Plan is optimal";
		break;
	case result::no_solution:
		buf << "Target function does not limit";
		break;
	case result::ceil_solution:
		break;
	case result::ERROR:
		break;
	}

	buf << "\n\n\n";

	file << buf.str() << std::flush;
}

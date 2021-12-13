#pragma once
#include <sstream>
#include <vector>
#include <memory>
#include <cmath>
#include "Plan.h"
#include "data.h"

enum class result 
{
	good_solution, 
	bad_solution, 
	no_solution,
	ceil_solution
};

class Simplex
{
public:
	Simplex(std::string filename = "") : outFile(std::move(filename)) {};
	Simplex(const Simplex&) = delete;
	Simplex& operator=(const Simplex&) = delete;
	
	std::shared_ptr<Plan> generate_plane(data::inputdata&);
	result run();

private:
	bool checkThColumn(const std::shared_ptr<Plan>&, const std::shared_ptr<Plan>&) const;
	result checkPlane(const std::shared_ptr<Plan>&) const;
	void dumpToTableTxt(const std::shared_ptr<Plan>&, size_t iteration, result, std::ofstream& file) const;
	void displayResult(const std::shared_ptr<Plan>&, size_t iteration, result) const;
	
	/*
	 * Выставляем базисные значения
	 * 
	 * */
	
	void setBasisVars(const std::shared_ptr<Plan>& source, std::shared_ptr<Plan>& target);

	/*
	 * Задаем значение координат при основных и базисных переменных
	*/

	void setFactorsOfVars(const std::shared_ptr<Plan>& source, std::shared_ptr<Plan>& target);

	/*
	 * Задаем значение в индексной таблице
	*/

	void setIndexString(const std::shared_ptr<Plan>& source, std::shared_ptr<Plan>& target);

	/*
	 * Задаем значения функции цели
	*/

	void setTargetFunction(const std::shared_ptr<Plan>& source, std::shared_ptr<Plan>& target);

	/*
	 * Задаем значения последнего столбца симплекс таблицы
	*/

	void setThColumn(std::shared_ptr<Plan>&);

	/*
	 * Задаем значения ведущего столбца
	*/

	void setIndexOfLeavingColumn(std::shared_ptr<Plan>&);

	/*
	 * Задаем значения ведущей строки
	*/

	void setIndexOfLeavingRow(std::shared_ptr<Plan>&);

	/*
	 * Задаем разрешающий элемент
	*/

	void setAllowingMember(std::shared_ptr<Plan>&);

	/*
	 * Все элементы свободного столбца должны быть неотрицательны, иначе система не имеет решения
	*/

	result checkNegative(std::shared_ptr<Plan>&);

private:
	const std::string outFile;
	
	std::shared_ptr<Plan> old_plane, new_plane;
	size_t numOfSourceVars;
	size_t numOfSourceRow;
	sysparser::Task wayOfTargetFunction;
	result res;
};

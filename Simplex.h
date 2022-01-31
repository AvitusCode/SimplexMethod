#pragma once
#include <sstream>
#include <vector>
#include <memory>
#include <cmath>
#include "Plan.h"
#include "data.h"

enum class result 
{
	ERROR,
	good_solution, 
	bad_solution, 
	no_solution,
	ceil_solution
};

enum class printMode
{
	PRINT,
	NPRINT
};

// Реализовать паттерн закрытого объявления и инициализации
class Simplex
{
public:
	Simplex(const Simplex&) = delete;
	Simplex& operator=(const Simplex&) = delete;
	
	// Реализовать сингелтон
	static Simplex* generate_plane(const data::inputdata& ud, const printMode& pm = printMode::PRINT, const std::string& filename = ""){
		if (simplex == nullptr)
		{
			simplex = new (std::nothrow) Simplex(pm, filename);
			if (simplex == nullptr){
				// Critical error. Not enough memory
				exit(-1);
			}
			simplex->generate(ud);
		}
		else{
			return simplex;
		}
		
		return simplex;
	}
	
	result run();

private:
	Simplex(printMode pm, std::string filename = "") : pm_(std::move(pm)), outFile(std::move(filename)) {};
	void generate(const data::inputdata& ud);
	
	bool checkThColumn(const std::shared_ptr<Plan>&, const std::shared_ptr<Plan>&) const;
	result checkPlane(const std::shared_ptr<Plan>&) const;
	void dumpToTableTxt(const std::shared_ptr<Plan>&, size_t iteration, result, std::ofstream& file) const;
	void displayResult(const std::shared_ptr<Plan>&, size_t iteration, result) const;
	
	/*
	 * Выставляем индексы базисных переменных, определяем новые значения вектора свободных членов
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
	 * Задаем значения Theta функции
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

private:
	const printMode pm_;
	const std::string outFile;
	inline static Simplex* simplex = nullptr;
	
	std::shared_ptr<Plan> old_plane, new_plane;
	size_t numOfSourceVars;
	size_t numOfSourceRow;
	sysparser::Task wayOfTargetFunction;
	result res;
};

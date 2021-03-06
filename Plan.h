#pragma once
#include <vector>
#include "Matrix.h"

struct Plan 
{
	 // Значение функции цели
	double targetFunction;

	//theta столбец для поиска разрешающего столбца
	std::vector<double> thColumn;

	//разрешающий элемент и его позиция в матрице
	size_t indexOfLeavingRow;
	size_t indexOfLeavingColumn;
	double allowingMember;

	std::vector<double> indexString; // Коэфф. функции цели (индексная строка)

	Matrix basisVars;   // Храним индексы базисных переменных и значения вектора своб членов
	Matrix varsFactors; // Коэффициенты системы ограничений (a_ij)

	Plan()
	{
		indexOfLeavingRow = 0;
		indexOfLeavingColumn = 0;
		allowingMember = 0;
		targetFunction = 0.0;
	}

	Plan(size_t vars, size_t rows)
	{
		indexOfLeavingRow = 0;
		indexOfLeavingColumn = 0;
		allowingMember = 0;
		targetFunction = 0.0;

		thColumn.resize(rows);
		basisVars.Reset(2, rows);
		varsFactors.Reset(rows, vars + rows);
		indexString.resize(vars + rows, 0);
	}

	Plan& operator=(const Plan& pl)
	{
		indexOfLeavingRow = pl.indexOfLeavingRow;
		indexOfLeavingColumn = pl.indexOfLeavingColumn;
		allowingMember = pl.allowingMember;
		targetFunction = pl.targetFunction;
		indexString.resize(pl.indexString.size(), 0);
		thColumn.resize(pl.thColumn.size());

		for (size_t i = 0; i < pl.indexString.size(); i++)
			indexString[i] = pl.indexString[i];
		for (size_t i = 0; i < pl.thColumn.size(); i++)
			thColumn[i] = pl.thColumn[i];

		basisVars = pl.basisVars.clone();
		varsFactors = pl.varsFactors.clone();

		return *this;
	}

	Plan(const Plan& pl){
		*this = pl;
	}

	Plan(Plan&&) = delete;

	~Plan()
	{
		thColumn.clear();
		indexString.clear();
		varsFactors.Reset(0, 0);
		basisVars.Reset(0, 0);
	}

};

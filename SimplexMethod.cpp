//============================================================================
// Name        : SimplexMethod.cpp
// Author      : AvitusCode
// Version     : 1.2
// Copyright   : free release
// Description : Simplex method in C++
//============================================================================

#include <iostream>
#include "Simplex.h"

// Дописать программу для работы с консолью (придумать граматику для пользовательской настройки)
// Написать cmake файл
// Добавить print mode для fileReader

int main(void) 
{
	data::inputdata dt; // Создаем структуру для данных задачи
	data::fileReader(dt, "matrix.txt", sysparser::Mode::WITHPARAMS); // заполняем структуру
	
	Simplex* simplex = Simplex::generate_plane(dt, printMode::PRINT, "log.txt"); // генерируем симплекс метод
	clearUserData(dt); // очищаем структуру
	simplex->run(); // запускаем процесс решения

	delete simplex; // освобождаем симплекс
	return 0;
}

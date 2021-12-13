//============================================================================
// Name        : SimplexMethod.cpp
// Author      : AvitusCode
// Version     : 1.0
// Copyright   : Your copyright notice
// Description : Simplex method in C++
//============================================================================

#include <iostream>
#include "Simplex.h"

// Придумать множество тестов, исследовать при крайних случаях, разобраться в симплекс-методе, подготовить программу к исследованиям

int main(void) 
{
	data::inputdata dt;
	data::fileReader(dt, "matrix.txt", sysparser::Mode::WITHPARAMS);
	
	Simplex simplex("log.txt");
	simplex.generate_plane(dt);
	simplex.run();

	return 0;
}

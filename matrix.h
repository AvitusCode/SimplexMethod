#pragma once
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <vector>

// Todo: make matrix template

class Matrix 
{
public:
    Matrix();
    Matrix(const Matrix& matrix);
    Matrix(size_t num_rows, size_t num_columns);
    Matrix& operator=(const Matrix& matrix);

    void Reset(size_t num_rows, size_t num_columns); // Make simple .assign();
    void Resize(size_t num_rows, size_t num_columns); // Make simple .resize();
    double& At(size_t row, size_t column);
    double At(size_t row, size_t column) const;
    size_t GetNumRows() const;
    size_t GetNumColumns() const;
    Matrix clone() const;

private:
    size_t num_rows_;
    size_t num_columns_;

    std::vector<std::vector<double>> elements_;
};

bool operator==(const Matrix& one, const Matrix& two);
Matrix operator+(const Matrix& one, const Matrix& two);
std::istream& operator>>(std::istream& in, Matrix& matrix);
std::ostream& operator<<(std::ostream& out, const Matrix& matrix);

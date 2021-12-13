#include "matrix.h"


Matrix::Matrix() : num_rows_(0), num_columns_(0) {}

Matrix::Matrix(size_t num_rows, size_t num_columns)
{
    Reset(num_rows, num_columns);
}

Matrix::Matrix(const Matrix& matrix)
{
	*this = matrix;
}
    
Matrix& Matrix::operator=(const Matrix& matrix)  
{
    Resize(matrix.GetNumRows(), matrix.GetNumColumns());
    
	for (size_t i = 0; i < num_rows_; i++)
		for (size_t j = 0; j < num_columns_; j++)
			elements_[i][j] = matrix.At(i, j);
    
	return *this;
}

void Matrix::Reset(size_t num_rows, size_t num_columns)
{
    if (num_rows == 0 || num_columns == 0) {
        num_rows = num_columns = 0;
    }

    num_rows_ = num_rows;
    num_columns_ = num_columns;
    elements_.assign(num_rows, std::vector<double>(num_columns, 0.0));
}

void Matrix::Resize(size_t num_rows, size_t num_columns)
{
    if (num_rows == 0 || num_columns == 0) {
        num_rows = num_columns = 0;
    }

    if (num_rows > num_rows_){
        elements_.push_back(std::vector<double>(num_columns - 1, 0.0));
    }

    while (num_rows < num_rows_)
    {
        elements_.pop_back();
        num_rows_--;
    }

    for (auto& vec : elements_)
        vec.resize(num_columns, 0.0);

    num_rows_ = num_rows;
    num_columns_ = num_columns;
}

double& Matrix::At(size_t row, size_t column) {
    return elements_.at(row).at(column);
}

double Matrix::At(size_t row, size_t column) const {
    return elements_.at(row).at(column);
}

size_t Matrix::GetNumRows() const {
    return num_rows_;
}

size_t Matrix::GetNumColumns() const {
    return num_columns_;
}

Matrix Matrix::clone() const
{
    Matrix result(num_rows_, num_columns_);

    for (size_t i = 0; i < num_rows_; i++)
        for (size_t j = 0; j < num_columns_; j++)
            result.At(i, j) = elements_.at(i).at(j);

    return result;
}

bool operator==(const Matrix& one, const Matrix& two) 
{
    if (one.GetNumRows() != two.GetNumRows())
        return false;
 

    if (one.GetNumColumns() != two.GetNumColumns())
        return false;
 

    for (size_t row = 0; row < one.GetNumRows(); ++row) 
    {
        for (size_t column = 0; column < one.GetNumColumns(); ++column) 
        {
            if (one.At(row, column) != two.At(row, column))
                return false;
        }
    }

    return true;
}

Matrix operator+(const Matrix& one, const Matrix& two)
{
    if (one.GetNumRows() != two.GetNumRows())
        throw std::invalid_argument("Mismatched number of rows");
   

    if (one.GetNumColumns() != two.GetNumColumns()) 
        throw std::invalid_argument("Mismatched number of columns");


    Matrix result(one.GetNumRows(), one.GetNumColumns());
    for (size_t row = 0; row < result.GetNumRows(); ++row) 
    {
        for (size_t column = 0; column < result.GetNumColumns(); ++column)
            result.At(row, column) = one.At(row, column) + two.At(row, column);
    }

    return result;
}

std::istream& operator>>(std::istream& in, Matrix& matrix) 
{
    size_t num_rows, num_columns;
    in >> num_rows >> num_columns;

    matrix.Reset(num_rows, num_columns);
    for (size_t row = 0; row < num_rows; ++row)
    {
        for (size_t column = 0; column < num_columns; ++column)
        {
            in >> matrix.At(row, column);
        }
    }

    return in;
}

std::ostream& operator<<(std::ostream& out, const Matrix& matrix)
{
    out << matrix.GetNumRows() << " " << matrix.GetNumColumns() << std::endl;
    for (size_t row = 0; row < matrix.GetNumRows(); ++row)
    {
        for (size_t column = 0; column < matrix.GetNumColumns(); ++column)
        {
            if (column > 0) 
                out << " ";
            out << matrix.At(row, column);
        }
        out << std::endl;
    }

    return out;
}

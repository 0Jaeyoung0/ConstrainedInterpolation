#include <Interpolate/Polynomial.h>

#include <cmath>


Polynomial::Polynomial()
{
    degree = -1;
}

Polynomial::Polynomial(const std::vector<double> coeff)
{
    this -> coeff = coeff;
    degree = coeff.size() - 1;
}

bool Polynomial::isInitialized()
{
    return degree >= 0;
}

void Polynomial::setCoeff(const std::vector<double> coeff)
{
    this -> coeff = coeff;
    degree = coeff.size() - 1;
}

std::vector<double> Polynomial::getCoeff()
{
    return coeff;
}

int Polynomial::getDegree()
{
    return degree;
}

std::optional<double> Polynomial::operator()(double x)
{
    return evaluate(x);
}

std::optional<double> Polynomial::evaluate(double x)
{
    if (!isInitialized())
    {
        return std::nullopt;
    }

    double result = 0.0;

    for (int i = 0; i < degree + 1; i++)
    {
        result += coeff[i] * pow(x, degree - i);
    }

    return result;
}

std::optional<double> Polynomial::firstDerivative(double x)
{
    if (!isInitialized())
    {
        return std::nullopt;
    }

    double result = 0.0;

    for(int i = 0; i < degree; i++)
    {
        result += (degree - i) * coeff[i] * pow(x, degree - i - 1);
    }

    return result;
}

std::optional<double> Polynomial::secondDerivative(double x)
{
    if (!isInitialized())
    {
        return std::nullopt;
    }

    double result = 0.0;

    for(int i = 0; i < degree - 1; i++)
    {
        result += (degree - i) * (degree - i - 1) * coeff[i] * pow(x, degree - i - 2);
    }

    return result;
}

#include <Interpolate/Explicit/ExplicitInterpolate.h>


std::vector<Polynomial> ExplicitInterpolate::getPolynomials()
{
    return polynomials;
}

std::optional<double> ExplicitInterpolate::operator()(double x)
{
    return evaluate(x);
}

std::optional<double> ExplicitInterpolate::evaluate(double x)
{
    for (int i = 0; i < x_points.size() - 1; i++)
    {
        if (isInRange(x, x_points[i], x_points[i + 1]))
        {
            return polynomials[i](x);
        }
    }
    
    return std::nullopt;
}

std::optional<double> ExplicitInterpolate::firstDerivative(double x)
{
    for (int i = 0; i < x_points.size() - 1; i++)
    {
        if (isInRange(x, x_points[i], x_points[i + 1]))
        {
            return polynomials[i].firstDerivative(x);
        }
    }
    
    return std::nullopt;
}

std::optional<double> ExplicitInterpolate::secondDerivative(double x)
{
    for (int i = 0; i < x_points.size() - 1; i++)
    {
        if (isInRange(x, x_points[i], x_points[i + 1]))
        {
            return polynomials[i].secondDerivative(x);
        }
    }
    
    return std::nullopt;
}

bool ExplicitInterpolate::isInRange(double value, double min, double max)
{   
    return (!(value < min) && !(value > max));
}

bool ExplicitInterpolate::isValidSize(int size)
{
    return size >= 2;
}

bool ExplicitInterpolate::isAscending(const std::vector<double>& values)
{
    for (int i = 0; i < values.size() - 1; i++)
    {
        if (values[i] > values[i + 1])
        {
            return false;
        }
    }

    return true;
}

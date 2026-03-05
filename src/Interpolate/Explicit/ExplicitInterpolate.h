#ifndef EXPLICIT_INTERPOLATE_H
#define EXPLICIT_INTERPOLATE_H


#include <Interpolate/Polynomial.h>


class ExplicitInterpolate
{
protected:
    std::vector<Polynomial> polynomials;
    std::vector<double> x_points;

public:
    virtual ~ExplicitInterpolate() = default;

    // getter
    std::vector<Polynomial> getPolynomials();

    // evaluate functions
    std::optional<double> operator()(double x);
    std::optional<double> evaluate(double x);
    std::optional<double> firstDerivative(double x);
    std::optional<double> secondDerivative(double x);

protected:
    ExplicitInterpolate() = default;

    bool isInRange(double value, double min, double max);
    static bool isValidSize(int size);
    static bool isAscending(const std::vector<double>& values);

    template <typename... Vectors>
    static bool isValidParameter(const std::vector<double>& x_points, const Vectors&... other_vectors)
    {
        if (!isValidSize(x_points.size()) || !isAscending(x_points))
        {
            return false;
        }

        if (((x_points.size() != other_vectors.size()) || ...))
        {
            return false;
        }

        return true;
    }
};


#endif

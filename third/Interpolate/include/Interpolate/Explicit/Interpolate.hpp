#ifndef INTERPOLATE_HPP
#define INTERPOLATE_HPP


#include <Interpolate/Polynomial.hpp>


void calcFirstDerivatives(const std::vector<double> &velocities, const std::vector<double> &thetas, std::vector<double> &x_first_derivative, std::vector<double> &y_first_derivative);
void calcSecondDerivatives(const std::vector<double> &x_first_derivative, const std::vector<double> &y_first_derivative, const std::vector<double> &curvatures, std::vector<double> &x_second_derivatives, std::vector<double> &y_second_derivatives);


class Interpolate
{
protected:
    std::vector<Polynomial> polynomials;
    std::vector<double> x_points;

public:
    virtual ~Interpolate() = default;

    // evaluate functions
    std::optional<double> operator()(double x);
    std::optional<double> evaluate(double x);
    std::optional<double> firstDerivative(double x);
    std::optional<double> secondDerivative(double x);

protected:
    Interpolate() = default;

    static bool isValidSize(int size);
    static bool isInRange(double value, double min, double max);
    static bool isAscending(const std::vector<double> &values);
};


#endif
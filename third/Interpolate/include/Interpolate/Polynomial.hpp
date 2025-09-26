#ifndef POLYNOMIAL_HPP
#define POLYNOMIAL_HPP


#include <vector>
#include <optional>


class Polynomial
{
private:
    std::vector<double> coeff;
    int degree;

public:
    Polynomial();
    Polynomial(const std::vector<double> coeff);

    bool isInitialized();

    // setter
    void setCoeff(const std::vector<double> coeff);

    // getter
    std::vector<double> getCoeff();
    int getDegree();

    // evalutate fuctions
    std::optional<double> operator()(double x);
    std::optional<double> evaluate(double x);
    std::optional<double> firstDerivative(double x);
    std::optional<double> secondDerivative(double x);
};


#endif
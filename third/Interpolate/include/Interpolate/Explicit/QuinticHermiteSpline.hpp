#ifndef QUINTIC_HERMITE_SPLINE_HPP
#define QUINTIC_HERMITE_SPLINE_HPP


#include <Interpolate/Explicit/Interpolate.hpp>

#include <Eigen/Dense>


class QuinticHermiteSpline : public Interpolate
{
public:
    static std::optional<QuinticHermiteSpline> create(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives, const std::vector<double>& second_derivatives);

private:
    QuinticHermiteSpline(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives, const std::vector<double>& second_derivatives);
    void constructMatrix(double x1, double x2, double y1, double y2, double y_prime1, double y_prime2, double y_double_prime1, double y_double_prime2, Eigen::MatrixXd& coeff_matrix, Eigen::VectorXd& rhs);
    std::vector<double> calcCoeff(double x1, double x2, double y1, double y2, double y_prime1, double y_prime2, double y_double_prime1, double y_double_prime2);
};


#endif
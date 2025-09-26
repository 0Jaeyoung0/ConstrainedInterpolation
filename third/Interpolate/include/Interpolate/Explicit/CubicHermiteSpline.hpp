#ifndef CUBIC_HERMITE_SPLINE_HPP
#define CUBIC_HERMITE_SPLINE_HPP


#include <Interpolate/Explicit/Interpolate.hpp>

#include <Eigen/Dense>


class CubicHermiteSpline : public Interpolate
{
public:
    static std::optional<CubicHermiteSpline> create(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives);

private:
    CubicHermiteSpline(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives);
    void constructMatrix(double x1, double x2, double y1, double y2, double y_prime1, double y_prime2, Eigen::MatrixXd& coeff_matrix, Eigen::VectorXd& rhs);
    std::vector<double> calcCoeff(double x1, double x2, double y1, double y2, double y_prime1, double y_prime2);
};


#endif
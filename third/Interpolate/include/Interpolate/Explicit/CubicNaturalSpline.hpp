#ifndef CUBIC_NATURAL_SPLINE_HPP
#define CUBIC_NATURAL_SPLINE_HPP


#include <Interpolate/Explicit/Interpolate.hpp>

#include <Eigen/Dense>


class CubicNaturalSpline : public Interpolate
{
public:
    static std::optional<CubicNaturalSpline> create(const std::vector<double>& x_points, const std::vector<double>& y_points, bool closed);

private:
    CubicNaturalSpline(const std::vector<double>& x_points, const std::vector<double>& y_points, bool closed);
    void constructMatrix(const std::vector<double>& x_points, const std::vector<double>& y_points, Eigen::MatrixXd& coeff_matrix, Eigen::VectorXd& rhs, bool closed);
    std::vector<double> calcCoeff(const std::vector<double>& x_points, const std::vector<double>& y_points, bool closed);
};


#endif
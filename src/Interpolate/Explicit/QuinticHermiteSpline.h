#ifndef QUINTIC_HERMITE_SPLINE_H
#define QUINTIC_HERMITE_SPLINE_H


#include <Interpolate/Explicit/ExplicitInterpolate.h>

#include <Eigen/Dense>


class QuinticHermiteSpline : public ExplicitInterpolate
{
public:
    static std::optional<QuinticHermiteSpline> create(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives, const std::vector<double>& second_derivatives);

protected:
    QuinticHermiteSpline(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives, const std::vector<double>& second_derivatives);

private:
    void constructMatrix(double x1, double x2, double y1, double y2, double y_prime1, double y_prime2, double y_double_prime1, double y_double_prime2, Eigen::MatrixXd& coeff_matrix, Eigen::VectorXd& rhs);
    std::vector<double> calcCoeff(double x1, double x2, double y1, double y2, double y_prime1, double y_prime2, double y_double_prime1, double y_double_prime2);
};


#endif

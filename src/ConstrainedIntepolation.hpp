#ifndef CONSTRAINED_INTERPOLATION_HPP
#define CONSTRAINED_INTERPOLATION_HPP


#include <Interpolate/Explicit/QuinticHermiteSpline.hpp>


class ConstrainedInterpolation
{
private:
    // Parametric QuinticHermiteSpline interpolation for C2 continuity.
    std::optional<QuinticHermiteSpline> x_spline;
    std::optional<QuinticHermiteSpline> y_spline;

public:
    static std::optional<ConstrainedInterpolation> create(const std::vector<double>& t, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& direction, const std::vector<double>& curvature);

    // evaluate functions
    std::optional<std::pair<double, double>> operator()(double t);
    std::optional<std::pair<double, double>> velocity(double t);
    std::optional<std::pair<double, double>> acceleration(double t);

    std::optional<std::pair<double, double>> evaluate(double t);
    std::optional<std::pair<double, double>> firstDerivative(double t);
    std::optional<std::pair<double, double>> secondDerivative(double t);

private:
    ConstrainedInterpolation(const std::vector<double>& t, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& direction, const std::vector<double>& curvature);

    double getRadian(double degree);

    // Calculate the speed values ​​at each control point using the Catmullom method.
    std::vector<double> calcSpeed(const std::vector<double>& t, const std::vector<double>& x, const std::vector<double>& y);

    // Calculate velocity using the speed and constrained direction values.
    void calcFirstDerivatives(const std::vector<double>& speed, const std::vector<double>& direction, std::vector<double>& x_prime, std::vector<double>& y_prime);

    // Calculate acceleration using velocity and curvature values.
    void calcSecondDerivatives(const std::vector<double>& x_prime, const std::vector<double>& y_prime, const std::vector<double>& speed, const std::vector<double>& curvature, std::vector<double>& x_double_prime, std::vector<double>& y_double_prime);
};


#endif
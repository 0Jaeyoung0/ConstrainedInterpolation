#ifndef QUINTIC_CATMULL_ROM_SPLINE_HPP
#define QUINTIC_CATMULL_ROM_SPLINE_HPP


#include <Interpolate/Explicit/QuinticHermiteSpline.h>


class QuinticCatmullRomSpline : public QuinticHermiteSpline
{
public:
    static std::optional<QuinticCatmullRomSpline> create(const std::vector<double>& x_points, const std::vector<double>& y_points);

private:
    QuinticCatmullRomSpline(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives, const std::vector<double>& second_derivatives);
    static std::vector<double> calcFirstDerivatives(const std::vector<double>& x_points, const std::vector<double>& y_points);
    static std::vector<double> calcSecondDerivatives(const std::vector<double>& x_points, const std::vector<double>& first_derivatives);
};


#endif

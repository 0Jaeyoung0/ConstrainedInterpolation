#ifndef CUBIC_CATMULL_ROM_SPLINE_HPP
#define CUBIC_CATMULL_ROM_SPLINE_HPP


#include <Interpolate/Explicit/CubicHermiteSpline.h>


class CubicCatmullRomSpline : public CubicHermiteSpline
{
public:
    static std::optional<CubicCatmullRomSpline> create(const std::vector<double>& x_points, const std::vector<double>& y_points);

private:
    CubicCatmullRomSpline(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives);
    static std::vector<double> calcFirstDerivatives(const std::vector<double>& x_points, const std::vector<double>& y_points);
};


#endif

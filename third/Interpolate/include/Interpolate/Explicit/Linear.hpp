#ifndef LINEAR_HPP
#define LINEAR_HPP


#include <Interpolate/Explicit/Interpolate.hpp>


class Linear : public Interpolate
{
public:
    static std::optional<Linear> create(const std::vector<double>& x_points, const std::vector<double>& y_points);

private:
    Linear(const std::vector<double>& x_points, const std::vector<double>& y_points);
    std::vector<double> calcCoeff(double x1, double x2, double y1, double y2);
};


#endif
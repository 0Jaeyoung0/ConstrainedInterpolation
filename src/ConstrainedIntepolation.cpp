#include "ConstrainedIntepolation.hpp"


std::optional<ConstrainedInterpolation> ConstrainedInterpolation::create(const std::vector<double>& t, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& direction, const std::vector<double>& curvature)
{
    if (t.size() != x.size() || t.size() != y.size() || t.size() != direction.size() || t.size() != curvature.size())
    {
        return std::nullopt;
    }
    if (t.size() < 2 || x.size() < 2 || y.size() < 2 || direction.size() < 2 || curvature.size() < 2)
    {
        return std::nullopt;
    }

    return ConstrainedInterpolation(t, x, y, direction, curvature);
}

std::optional<std::pair<double, double>> ConstrainedInterpolation::operator()(double t)
{
    return evaluate(t);
}

std::optional<std::pair<double, double>> ConstrainedInterpolation::velocity(double t)
{
    return firstDerivative(t);
}

std::optional<std::pair<double, double>> ConstrainedInterpolation::acceleration(double t)
{
    return secondDerivative(t);
}

std::optional<std::pair<double, double>> ConstrainedInterpolation::evaluate(double t)
{
    auto x_result = x_spline -> evaluate(t);
    auto y_result = y_spline -> evaluate(t);

    if (x_result.has_value() && y_result.has_value())
    {
        return std::make_pair(x_result.value(), y_result.value());
    }
    else
    {
        return std::nullopt;
    }
}

std::optional<std::pair<double, double>> ConstrainedInterpolation::firstDerivative(double t)
{
    auto x_result = x_spline -> firstDerivative(t);
    auto y_result = y_spline -> firstDerivative(t);

    if (x_result.has_value() && y_result.has_value())
    {
        return std::make_pair(x_result.value(), y_result.value());
    }
    else
    {
        return std::nullopt;
    }
}

std::optional<std::pair<double, double>> ConstrainedInterpolation::secondDerivative(double t)
{
    auto x_result = x_spline -> secondDerivative(t);
    auto y_result = y_spline -> secondDerivative(t);

    if (x_result.has_value() && y_result.has_value())
    {
        return std::make_pair(x_result.value(), y_result.value());
    }
    else
    {
        return std::nullopt;
    }
}

ConstrainedInterpolation::ConstrainedInterpolation(const std::vector<double>& t, const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& direction, const std::vector<double>& curvature)
{
    std::vector<double> speed = calcSpeed(t, x, y);

    std::vector<double> x_prime;
    std::vector<double> y_prime;
    calcFirstDerivatives(speed, direction, x_prime, y_prime);

    std::vector<double> x_double_prime;
    std::vector<double> y_double_prime;
    calcSecondDerivatives(x_prime, y_prime, speed, curvature, x_double_prime, y_double_prime);

    x_spline = QuinticHermiteSpline::create(t, x, x_prime, x_double_prime);
    y_spline = QuinticHermiteSpline::create(t, y, y_prime, y_double_prime);
}

double ConstrainedInterpolation::getRadian(double degree)
{
    return degree * M_PI / 180.0;
}

std::vector<double> ConstrainedInterpolation::calcSpeed(const std::vector<double>& t, const std::vector<double>& x, const std::vector<double>& y)
{
    std::vector<double> result;
    result.reserve(x.size());

    for (int i = 0; i < x.size(); i++) 
    {
        double dx;
        double dy;

        if (i == 0)
        {
            dx = (x[1] - x[0]) / (t[1] - t[0]);
            dy = (y[1] - y[0]) / (t[1] - t[0]);
        }
        else if (i == x.size() - 1)
        {
            dx = (x[i] - x[i - 1]) / (t[i] - t[i - 1]);
            dy = (y[i] - y[i - 1]) / (t[i] - t[i - 1]);
        }
        else
        {
            dx = (x[i + 1] - x[i - 1]) / (t[i + 1] - t[i - 1]);
            dy = (y[i + 1] - y[i - 1]) / (t[i + 1] - t[i - 1]);
        }

        double speed = sqrt(dx * dx + dy * dy) / 2;
        result.push_back(speed);
    }

    return result;
}

void ConstrainedInterpolation::calcFirstDerivatives(const std::vector<double>& speed, const std::vector<double>& direction, std::vector<double>& x_prime, std::vector<double>& y_prime)
{
    x_prime.reserve(speed.size());
    y_prime.reserve(speed.size());

    for (int i = 0; i < speed.size(); i++) 
    {
        x_prime.push_back(speed[i] * cos(getRadian(direction[i])));
        y_prime.push_back(speed[i] * sin(getRadian(direction[i])));
    }
}

void ConstrainedInterpolation::calcSecondDerivatives(const std::vector<double>& x_prime, const std::vector<double>& y_prime, const std::vector<double>& speed, const std::vector<double>& curvature, std::vector<double>& x_double_prime, std::vector<double>& y_double_prime)
{
    x_double_prime.reserve(x_prime.size());
    y_double_prime.reserve(x_prime.size());

    for(int i = 0; i < x_prime.size(); i++)
    {
        double nomal_acceleration = speed[i] * speed[i] * curvature[i];

        double nomal_unit_x = -y_prime[i] / speed[i];
        double nomal_unit_y =  x_prime[i] / speed[i];

        double nomal_x = nomal_acceleration * nomal_unit_x;
        double nomal_y = nomal_acceleration * nomal_unit_y;

        x_double_prime.push_back(nomal_x);
        y_double_prime.push_back(nomal_y);
    }
}
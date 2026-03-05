#include <Interpolate/Explicit/QuinticHermiteSpline.h>


std::optional<QuinticHermiteSpline> QuinticHermiteSpline::create(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives, const std::vector<double>& second_derivatives)
{
    if (!isValidParameter(x_points, y_points, first_derivatives, second_derivatives))
    {
        return std::nullopt;
    }

    return QuinticHermiteSpline(x_points, y_points, first_derivatives, second_derivatives);
}

QuinticHermiteSpline::QuinticHermiteSpline(const std::vector<double>& x_points, const std::vector<double>& y_points, const std::vector<double>& first_derivatives, const std::vector<double>& second_derivatives)
{
    this -> x_points = x_points;
    
    int point_num = x_points.size();
    int poly_num = point_num - 1;

    for (int i = 0; i < poly_num; i++)
    {
        std::vector<double> coeff = calcCoeff(x_points[i], x_points[i + 1], y_points[i], y_points[i + 1], first_derivatives[i], first_derivatives[i + 1], second_derivatives[i], second_derivatives[i + 1]);
        polynomials.emplace_back(coeff);
    }
}

void QuinticHermiteSpline::constructMatrix(double x1, double x2, double y1, double y2, double y_prime1, double y_prime2, double y_double_prime1, double y_double_prime2, Eigen::MatrixXd& coeff_matrix, Eigen::VectorXd& rhs)
{
    // f(x_1) = y_1
    coeff_matrix(0, 0) = pow(x1, 5);
    coeff_matrix(0, 1) = pow(x1, 4);
    coeff_matrix(0, 2) = pow(x1, 3);
    coeff_matrix(0, 3) = pow(x1, 2);
    coeff_matrix(0, 4) = x1;
    coeff_matrix(0, 5) = 1.0;

    rhs(0) = y1;

    // f(x_2) = y_2
    coeff_matrix(1, 0) = pow(x2, 5);
    coeff_matrix(1, 1) = pow(x2, 4);
    coeff_matrix(1, 2) = pow(x2, 3);
    coeff_matrix(1, 3) = pow(x2, 2);
    coeff_matrix(1, 4) = x2;
    coeff_matrix(1, 5) = 1.0;

    rhs(1) = y2;

    // f'(x_1) = y_prime1
    coeff_matrix(2, 0) = 5.0 * pow(x1, 4);
    coeff_matrix(2, 1) = 4.0 * pow(x1, 3);
    coeff_matrix(2, 2) = 3.0 * pow(x1, 2);
    coeff_matrix(2, 3) = 2.0 * x1;
    coeff_matrix(2, 4) = 1.0;
    coeff_matrix(2, 5) = 0.0;

    rhs(2) = y_prime1;

    // f'(x_2) = y_prime2
    coeff_matrix(3, 0) = 5.0 * pow(x2, 4);
    coeff_matrix(3, 1) = 4.0 * pow(x2, 3);
    coeff_matrix(3, 2) = 3.0 * pow(x2, 2);
    coeff_matrix(3, 3) = 2.0 * x2;
    coeff_matrix(3, 4) = 1.0;
    coeff_matrix(3, 5) = 0.0;

    rhs(3) = y_prime2;

    // f''(x_1) = y_double_prime1
    coeff_matrix(4, 0) = 20.0 * pow(x1, 3);
    coeff_matrix(4, 1) = 12.0 * pow(x1, 2);
    coeff_matrix(4, 2) = 6.0 * x1;
    coeff_matrix(4, 3) = 2.0;
    coeff_matrix(4, 4) = 0.0;
    coeff_matrix(4, 5) = 0.0;

    rhs(4) = y_double_prime1;

    // f''(x_2) = y_double_prime2
    coeff_matrix(5, 0) = 20.0 * pow(x2, 3);
    coeff_matrix(5, 1) = 12.0 * pow(x2, 2);
    coeff_matrix(5, 2) = 6.0 * x2;
    coeff_matrix(5, 3) = 2.0;
    coeff_matrix(5, 4) = 0.0;
    coeff_matrix(5, 5) = 0.0;

    rhs(5) = y_double_prime2;
}

std::vector<double> QuinticHermiteSpline::calcCoeff(double x1, double x2, double y1, double y2, double y_prime1, double y_prime2, double y_double_prime1, double y_double_prime2)
{
    Eigen::MatrixXd coeff_matrix(6, 6);
    Eigen::VectorXd rhs(6);

    constructMatrix(x1, x2, y1, y2, y_prime1, y_prime2, y_double_prime1, y_double_prime2, coeff_matrix, rhs);

    Eigen::HouseholderQR<Eigen::MatrixXd> qr(coeff_matrix);
    Eigen::VectorXd coeff = qr.solve(rhs);

    std::vector<double> coeff_1d(coeff.data(), coeff.data() + coeff.size());

    return coeff_1d;
}

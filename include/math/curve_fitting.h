#pragma once

#include <array>

#include <Eigen/Dense>
template<typename T, unsigned int N>
Eigen::Matrix<T, N, N> PseudoInverse(const Eigen::Matrix<T, N, N> &m,
                                     const double epsilon = 1.0e-6) {
    Eigen::JacobiSVD<Eigen::Matrix<T, N, N>> svd(
            m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return static_cast<Eigen::Matrix<T, N, N>>(svd.matrixV() *
                                               (svd.singularValues().array().abs() > epsilon)
                                                       .select(svd.singularValues().array().inverse(), 0)
                                                       .matrix()
                                                       .asDiagonal() *
                                               svd.matrixU().adjoint());
}

template<typename T, unsigned int M, unsigned int N>
Eigen::Matrix<T, N, M> PseudoInverse(const Eigen::Matrix<T, M, N> &m,
                                     const double epsilon = 1.0e-6) {
    Eigen::Matrix<T, M, M> t = m * m.transpose();
    return static_cast<Eigen::Matrix<T, N, M>>(m.transpose() *
                                               PseudoInverse<T, M>(t));
}

// The coef is in ascending order,
// i.e., f(x) = coef[0] + coef[1] * x + coef[2] * x^2 ...
template<std::size_t N>
double EvaluatePolynomial(const std::array<double, N + 1> &coef,
                          const double p) {
    double r = 0.0;
    for (int i = N; i >= 0; --i) {
        r = r * p + coef[i];
    }
    return r;
}

// Fit a Nth order polynomial
template<std::size_t N>
std::array<double, N + 1> FitPolynomial(
        const std::vector<Eigen::Vector2f>& points,
        double *ptr_error_square = nullptr) {
    const int data_size = points.size();
    Eigen::MatrixXd X(data_size, N + 1);
    Eigen::MatrixXd Y(data_size, 1);
    for (std::size_t i = 0; i < points.size(); ++i) {

        double x = points[i][0];
        double y = points[i][1];

        X(i, 0) = 1.0;
        for (std::size_t j = 1; j < N + 1; ++j) {
            X(i, j) = X(i, j - 1) * x;
        }

        Y(i, 0) = y;
    }


    Eigen::Matrix<double, N + 1, 1> t =
            PseudoInverse<double, N + 1, N + 1>(X.transpose() * X) *
            X.transpose() * Y;

    std::array<double, N + 1> coefs;
    for (std::size_t i = 0; i < N + 1; ++i) {
        coefs[i] = t(i, 0);
    }

    if (ptr_error_square != nullptr) {
        *ptr_error_square = 0.0;
        for (const auto &point : points) {
            double error = EvaluatePolynomial<N>(coefs, point[0]) - point[1];
            *ptr_error_square += error * error;
        }
    }
    return coefs;
}

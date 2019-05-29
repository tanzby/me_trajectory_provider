#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "Singleton.h"


class Plotter {
private:

    cv::Mat image;

    double resolution_;
    double x_shift_;
    double y_shift_;

    Plotter(int width, int height, double reselution, double x_shift, double y_shift) {
        resolution_ = reselution;
        x_shift_ = x_shift;
        y_shift_ = y_shift;
        image = cv::Mat(height, width, CV_8UC4);
        cv::namedWindow("LM_Plotter");
    }

    DISABLE_COPY_AND_ASSIGN(Plotter);

public:

    struct Color {
        unsigned char red;
        unsigned char green;
        unsigned char blue;
        unsigned char alpha;

        Color() : red(255), green(255), blue(255), alpha(255) {}

        Color(const unsigned char &r, const unsigned char &g, const unsigned char &b, const unsigned char &a) :
                red(r), green(g), blue(b), alpha(a) {}

    };

    static Plotter *Instance(int width = 480, int height = 480, double reselution = 1.0, double x_shift = 0.0,
                             double y_shift = 0.0) {
        static Plotter Instance(width, height, reselution, x_shift, y_shift);
        return &Instance;
    }

    void clear() {
        image = 0;
    }

    void plot_text(int x, int y, const std::string &content) {
        static std::string prev_content = "";
        cv::putText(image, prev_content, {x, y}, cv::QT_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0));
        cv::putText(image, content, {x, y}, cv::QT_FONT_NORMAL, 0.5, cv::Scalar(255, 255, 255));
        prev_content = content;
    }

    inline void plot(const Eigen::Vector2f& point, const Color& color, int thickness = 1) {

        int row_(image.rows - (point[1] + y_shift_) / resolution_);
        int col_((point[0] + x_shift_) / resolution_);

        if (thickness == 0) {
            if (row_ < 0 || row_ >= image.rows || col_ < 0 || col_ >= image.cols) {

            } else{
                auto &cell = image.at<cv::Vec4b>(row_, col_);
                cell[0] = color.blue;
                cell[1] = color.green;
                cell[2] = color.red;
                cell[3] = color.alpha;
            }
        } else {
            cv::circle(image, {col_, row_}, thickness,
                       cv::Scalar(color.blue, color.green, color.red, color.alpha), -1);
        }
    }

    void plot(const std::deque<Eigen::Vector2f> &points, const Color& color, int thickness = 1) {

        for (const auto &point: points) {
            this->plot(point, color, thickness);
        }
    }

    void plot(const std::vector<Eigen::Vector2f> &points,const Color& color, int thickness = 1) {

        for (const auto &point: points) {
            this->plot(point, color, thickness);
        }
    }



    void plot_polynomial(const std::vector<double> &coef, Color color, int thickness = 1) {
        std::vector<cv::Point> points;

        for (int x = 0; x < image.cols; ++x) {
            // (real_x, real_y) => ((real_x + shift_x)/r, ... )
            double y = 0.0;
            double real_x = x * resolution_ - x_shift_;

            for (int i = coef.size() - 1; i >= 0; --i) {
                y = y * real_x + coef[i];
            }

            y = (y + y_shift_) / resolution_;

            if (y >= image.rows || y < 0) {
                continue;
            }

            points.emplace_back(x, image.rows - y);
        }

        cv::polylines(image, points, false, cv::Scalar(color.blue, color.green, color.red, color.alpha), thickness,
                      cv::LineTypes::LINE_AA);
    }

    void show(bool key_block = true) {
        cv::imshow("LM_Plotter", image);
        cv::waitKey(key_block ? 0 : 1);
    }

    void set_shift(double x_shift, double y_shift) {
        this->x_shift_ = x_shift;
        this->y_shift_ = y_shift;
    }
};
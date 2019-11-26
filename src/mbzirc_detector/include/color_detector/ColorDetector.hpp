#pragma once

#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "BBox.hpp"


class ColorDetector {
    unsigned l_min;
    unsigned l_max;
    unsigned a_min;
    unsigned a_max;
    unsigned b_min;
    unsigned b_max;
    cv::Scalar lb;
    cv::Scalar ub;
    std::string calib_dir;
    bool single_main_target;
    double min_area_pix;
    unsigned max_objects;
    unsigned hu_metric;
    unsigned hu_soft_hard_area_thresh;
    double hu_max_dist_soft;
    double hu_max_dist_hard;
    std::vector<cv::Point> circle_con;

    void setColorThresholds(const std::vector<unsigned> &th);
    void loadCircleContour();
public:
    ColorDetector(std::vector<unsigned> th, double min_area_pix,  unsigned max_objects, 
        std::string calib_dir, bool single_main_target, unsigned hu_metric, 
        unsigned hu_soft_hard_area_thresh, double hu_max_dist_soft, double hu_max_dist_hard);
    ~ColorDetector();
    std::vector<BBox> detect(const cv::Mat &img);
};

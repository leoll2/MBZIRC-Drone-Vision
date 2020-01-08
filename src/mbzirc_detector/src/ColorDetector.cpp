#include <assert.h>
#include <iostream>
#include <tuple>

#include "color_detector/ColorDetector.hpp"
#include "color_detector/cv_algorithms.hpp"


/* Set color thresholds values */
void ColorDetector::setColorThresholds(const std::vector<unsigned> &th)
{
    assert(th.size() == 6);
    this->l_min = th[0];
    this->l_max = th[1];
    this->a_min = th[2];
    this->a_max = th[3];
    this->b_min = th[4];
    this->b_max = th[5];

    this->lb = cv::Scalar(l_min, a_min, b_min);
    this->ub = cv::Scalar(l_max, a_max, b_max);
}


/* Load the circle calibration image and get its contour, which will be later
 * used to extrapolate Hu moments */
void ColorDetector::loadCircleContour()
{
    cv::Mat circle_img, circle_lab, circle_mask;

    circle_img = cv::imread(calib_dir + "circle.png", cv::IMREAD_COLOR);
    if (!circle_img.data) {
        std::cerr << "Missing calibration file (circle.png)" << std::endl;
        return;
    }
    circle_lab = cv_alg::bgr2lab(circle_img);
    // Note: the calibration image is red (always!), so appropriate thresholds are needed
    cv::Scalar lb_calib = cv::Scalar(0, 160, 130);
    cv::Scalar ub_calib = cv::Scalar(255, 255, 255);
    circle_mask = cv_alg::LabColorFilter(circle_lab, &lb_calib, &ub_calib);
    this->circle_con = std::vector<cv::Point>(cv_alg::findContours(circle_mask)[0]);
}


/* Constructor */
ColorDetector::ColorDetector(std::vector<unsigned> th, double min_area_pix,
    unsigned max_objects, std::string calib_dir, bool single_main_target, unsigned hu_metric, 
    unsigned hu_soft_hard_area_thresh, double hu_max_dist_soft, double hu_max_dist_hard)
{
    this->min_area_pix = min_area_pix;
    this->max_objects = max_objects;
    this->calib_dir = calib_dir;
    this->single_main_target = single_main_target;
    this->hu_metric = hu_metric;
    this->hu_soft_hard_area_thresh = hu_soft_hard_area_thresh;
    this->hu_max_dist_soft = hu_max_dist_soft;
    this->hu_max_dist_hard = hu_max_dist_hard;

    setColorThresholds(th);

    loadCircleContour();
}


/* Destructor */
ColorDetector::~ColorDetector() {}


bool compareContAreaPairs(std::pair<std::vector<cv::Point>, double> a,
    std::pair<std::vector<cv::Point>, double> b)
{
    return a.second > b.second;
}


bool compareContRoundAreaTuples(std::tuple<std::vector<cv::Point>, double, double> a,
    std::tuple<std::vector<cv::Point>, double, double> b)
{
    // area by roundness ratio (higher -> better)
    return (std::get<2>(a) / std::get<1>(a)) > (std::get<2>(b) / std::get<1>(b));
}


/* Detect objects in the image */
std::vector<BBox> ColorDetector::detect(const cv::Mat &img)
{
    std::vector<BBox> res_bboxes;
    std::vector<std::tuple<std::vector<cv::Point>, double, double>> circular_cnts;

    // Compute frame center (pixel)
    unsigned frame_cx = img.cols/2;
    unsigned frame_cy = img.rows/2;

    // Convert to L*a*b* color space
    cv::Mat frame_lab = cv_alg::bgr2lab(img);

    // Apply color filter (mask)
    cv::Mat color_mask = cv_alg::LabColorFilter(frame_lab, &lb, &ub);

    // Find the contours
    std::vector<std::vector<cv::Point>> contours = cv_alg::findContours(color_mask);

    if (contours.size() > 0) {

        // Map contours to their area, filtering out irrelevant ones
        std::vector<std::pair<std::vector<cv::Point>, double>> cnts_area;
        for (const auto &c : contours) {
            double area = cv::contourArea(c);
            if (area > this->min_area_pix)
                cnts_area.push_back(std::make_pair(c, cv::contourArea(c)));
        }

        // Sort by area (largest first)
        std::sort(cnts_area.begin(), cnts_area.end(), compareContAreaPairs);

        // Truncate, keeping only the largest
        if (cnts_area.size() > this->max_objects)
            cnts_area.erase(cnts_area.begin() + this->max_objects, cnts_area.end());

        // Isolate pseudo-circular contours (roundness close to 0)
        for (const auto &ca : cnts_area) {
            // Compute bounding box around the contour
            cv::Rect roi = cv::boundingRect(ca.first);
            // Compute its roundness
            double roundness = cv::matchShapes(ca.first, circle_con, hu_metric, 0.0);
            if ((roundness < hu_max_dist_hard) ||
                (ca.second < hu_soft_hard_area_thresh && roundness < hu_max_dist_soft))
            {
                std::tuple<std::vector<cv::Point>, double, double> cra(ca.first, roundness, ca.second);
                circular_cnts.push_back(cra);
            }
        }

        // Sort contours by area/roundness ratio
        std::sort(circular_cnts.begin(), circular_cnts.end(), compareContRoundAreaTuples);

        if (single_main_target && (circular_cnts.size() > 1)) {
            circular_cnts.erase(circular_cnts.begin() + 1, circular_cnts.end());
        }

        for (const auto &cra : circular_cnts) {
            BBox b;
            cv::Rect roi = cv::boundingRect(std::get<0>(cra));
            b.x = roi.x;
            b.y = roi.y;
            b.w = roi.width;
            b.h = roi.height;
            b.prob = 0.5;   // TODO
            b.obj_class = "ball_red";
            res_bboxes.push_back(b);
        }
    }
    return res_bboxes;
}

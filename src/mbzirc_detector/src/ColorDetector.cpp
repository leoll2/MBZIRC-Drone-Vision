#include <assert.h>

#include "color_detector/ColorDetector.hpp"
#include "color_detector/cv_algorithms.hpp"


/* Set color thresholds values */
void ColorDetector::setColorThresholds(const std::vector<unsigned> &th)
{
    assert(th.size() == 6);
    this->h_min = th[0];
    this->h_max = th[1];
    this->s_min = th[2];
    this->s_max = th[3];
    this->v_min = th[4];
    this->v_max = th[5];

    this->lb1 = cv::Scalar(h_min, s_min, v_min);
    this->ub1 = cv::Scalar(h_max, s_max, v_max);
    this->lb2 = cv::Scalar(180-h_max, s_min, v_min);
    this->ub2 = cv::Scalar(180-h_min, s_max, v_max);
}


/* Constructor */
ColorDetector::ColorDetector(std::vector<unsigned> th, double min_area_pix,
    unsigned max_objects)
{
    this->min_area_pix = min_area_pix;
    setColorThresholds(th);
    // TODO: setContour
}

/* Destructor */
ColorDetector::~ColorDetector() {}


// TODO loadContour()


bool compareContAreaPairs(std::pair<std::vector<cv::Point>, double> a,
    std::pair<std::vector<cv::Point>, double> b)
{
    return a.second > b.second;
}


/* Detect objects in the image */
std::vector<BBox> ColorDetector::detect(const cv::Mat &img)
{
    std::vector<BBox> res_bboxes;

    // TODO scale_frame (optional)

    // Compute frame center (pixel)
    unsigned frame_cx = img.cols/2;
    unsigned frame_cy = img.rows/2;

    // Convert to HSV
    cv::Mat frame_hsv = cv_alg::bgr2hsv(img);

    // Apply color filter (mask)
    cv::Mat color_mask = cv_alg::colorFilter(frame_hsv, &lb1, &ub1, &lb2, &ub2);

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

        for (const auto &ca : cnts_area) {
            BBox b;
            cv::Rect roi = cv::boundingRect(ca.first);
            //cv::Rect roi = cv_alg::resizeBox(brect, 3);
            b.x = roi.x;
            b.y = roi.y;
            b.w = roi.width;
            b.h = roi.height;
            b.prob = 0.5;   // TODO
            b.obj_class = "ball";
            res_bboxes.push_back(b);
        }
    }
    return res_bboxes;
}

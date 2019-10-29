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
ColorDetector::ColorDetector(std::vector<unsigned> th)
{
    setColorThresholds(th);
    // TODO: setContour
}

/* Destructor */
ColorDetector::~ColorDetector() {}


// TODO loadContour()


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

    // TODO assign result
    return res_bboxes;
}

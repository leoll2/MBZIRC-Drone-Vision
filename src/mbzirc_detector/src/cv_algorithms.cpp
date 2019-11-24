#include <opencv2/imgproc.hpp>

#include "color_detector/cv_algorithms.hpp"

namespace cv_alg {

    /* Blurs an image using a Gaussian filter
     * Convolution between image and Gaussian kernel.
     * Parameters
     * ----------
     * ksize : (int, int) odd
     *     Kernel size
     * sigma : double, optional
     *     Gaussian standard deviation
     */
    cv::Mat gaussianBlur(const cv::Mat &src, cv::Size ksize, double sigma)
    {
        cv::Mat dst;
        cv::GaussianBlur(src, dst, ksize, sigma);
        return dst;
    }


    /* Applies the bilateral filter to an image
     * Smoothing with edge-preservation.
     * Parameters
     * ----------
     * d : int
     *     Neighborhood diameter.
     *     Note: flattens color, but also makes transparent. More than 10 is heavy computationally.
     * sigmacolor : int
     *     Filter sigma in color space. Higher value mixes farther colors together
     * sigmaspace: int
     *     Filter sigma in coord space. Higher values make farther points interact more.
     *     Note: not so relevant in practice
     */
     cv::Mat bilateralFilter(const cv::Mat &src, int d, int sigmacolor, int sigmaspace)
     {
        cv::Mat dst;
        cv::bilateralFilter(src, dst, d, sigmacolor, sigmaspace);
        return dst;
     }


    /* Meanshift segmentation
     * Parameters
     * ----------
     * sp : int
     *     Spatial activation radius
     * sr : int
     *     Color activation threshold
     */
    cv::Mat pyramidFilter(const cv::Mat &src, int sp, int sr)
    {
        cv::Mat dst;
        cv::pyrMeanShiftFiltering(src, dst, sp, sr);
        return dst;
    }


    /*************************************/
    /*        COLOR TRANSFORMATIONS      */
    /*************************************/


    /* Convert a frame from BGR to HSV */
    cv::Mat bgr2hsv(const cv::Mat &src)
    {
        cv::Mat dst;
        cvtColor(src, dst, cv::COLOR_BGR2HSV);
        return dst;
    }


    /* Convert a frame from BGR to CIE L*a*b* */
    cv::Mat bgr2lab(const cv::Mat &src)
    {
        cv::Mat dst;
        cvtColor(src, dst, cv::COLOR_BGR2Lab);
        return dst;
    }


    /* Compute HSV color mask based on the specified thresholds.
     * It is possible to specify two ranges, which is especially
     * useful when thresholding around 0 (e.g. red color)
     *
     * Note: the src frame is assumed to be already in HSV format.
     * Parameters
     * ----------
     * lb1 : Scalar
     *      HSV lower bound #1
     * ub1 : Scalar
     *      HSV upper bound #1
     * lb2 : Scalar (optional)
     *      HSV lower bound #2
     * ub2 : Scalar (optional)
     *      HSV upper bound #2
     * 
     */
    cv::Mat HSVColorFilter(const cv::Mat &src, 
        const cv::Scalar *lb1, const cv::Scalar *ub1, 
        const cv::Scalar *lb2, const cv::Scalar *ub2)
    {
        cv::Mat dst;

        if ((lb2 == nullptr) || (ub2 == nullptr)) {
            // single threshold
            cv::inRange(src, *lb1, *ub1, dst);
        } else {
            // double threshold
            cv::Mat mask1, mask2;
            cv::inRange(src, *lb1, *ub1, mask1);
            cv::inRange(src, *lb2, *ub2, mask2);
            dst = mask1 | mask2;
        }
        return dst;
    }


    /* Compute L*a*b* color mask based on the specified thresholds.
     *
     * Note: the src frame is assumed to be already in L*a*b* format.
     * Parameters
     * ----------
     * lb : Scalar
     *      Lab lower bound
     * ub : Scalar
     *      Lab upper bound
     */
    cv::Mat LabColorFilter(const cv::Mat &src, 
        const cv::Scalar *lb, const cv::Scalar *ub) 
    {
        cv::Mat dst;
        cv::inRange(src, *lb, *ub, dst);
        return dst;
    }


    /* Compute the opposite HSV color mask of the specified thresholds.
     * It is also possible to specify a soft margin (thresholds are loosened)
     * 
     * Note: the src frame is assumed to be already in HSV format.
     */
    cv::Mat negHSVColorFilter(const cv::Mat &src, 
        const cv::Scalar *lb1, const cv::Scalar *ub1, 
        const cv::Scalar *lb2, const cv::Scalar *ub2, unsigned margin)
    {
        cv::Scalar extlb1, extub1, extlb2, extub2;
        cv::Mat extmask, dst;

        extlb1 = *lb1 - cv::Scalar(margin);
        if (extlb1[0] < 0) extlb1[0] = 0;   // prevent underflow
        if (extlb1[1] < 0) extlb1[1] = 0;
        if (extlb1[2] < 0) extlb1[2] = 0;

        extub1 = *ub1 + cv::Scalar(margin);
        if (extub1[0] > 180) extub1[0] = 180; // prevent overflow
        if (extub1[1] > 255) extub1[1] = 255;
        if (extub1[2] > 255) extub1[2] = 255;

        if (lb2 != nullptr) {
            extlb2 = *lb2 - cv::Scalar(margin);
            if (extlb2[0] < 0) extlb2[0] = 0;   // prevent underflow
            if (extlb2[1] < 0) extlb2[1] = 0;
            if (extlb2[2] < 0) extlb2[2] = 0;
            extub2 = *ub2 + cv::Scalar(margin);
            if (extub2[0] > 180) extub2[0] = 180; // prevent overflow
            if (extub2[1] > 255) extub2[1] = 255;
            if (extub2[2] > 255) extub2[2] = 255;
            extmask = HSVColorFilter(src, &extlb1, &extub1, &extlb2, &extub2);
        } else {
            extmask = HSVColorFilter(src, &extlb1, &extub1, nullptr, nullptr);
        }

        cv::bitwise_not(extmask, dst);
        return dst;
    }


    /* Detect contours in a binary image */
    std::vector<std::vector<cv::Point>> findContours(const cv::Mat &src)
    {
        std::vector<std::vector<cv::Point>> ctr;
        cv::findContours(src, ctr, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        return ctr;
    }


    /* Resize a box from the center.
     * Note: input values get modified
     * Parameters
     * ----------
     * r : unsigned
     *    initial Rectangle
     * scale : unsigned
     *    scaling factor
     */
    cv::Rect resizeBox(const cv::Rect &r, const unsigned scale)
    {
        cv::Rect res_box;

        res_box.x = r.x + r.width/2;
        res_box.y = r.y + r.height/2;
        res_box.width = r.width * scale;
        res_box.height = r.height * scale;

        res_box.x = std::max(0, res_box.x - res_box.width/2);
        res_box.y = std::max(0, res_box.y - res_box.height/2);

        return res_box;
    }
}

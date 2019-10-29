#pragma once

namespace cv_alg {
    
    cv::Mat gaussianBlur(const cv::Mat &src, cv::Size ksize=cv::Size(3, 3), double sigma=0.);

    cv::Mat bilateralFilter(const cv::Mat &src, int d=9, int sigmacolor=75, int sigmaspace=10);

    cv::Mat pyramidFilter(const cv::Mat &src, int sp=10, int sr=30);

    cv::Mat bgr2hsv(const cv::Mat &src);

    cv::Mat colorFilter(const cv::Mat &src,
        const cv::Scalar *lb1, const cv::Scalar *ub1,
        const cv::Scalar *lb2=nullptr, const cv::Scalar *ub2=nullptr
    );
    
    cv::Mat negColorFilter(const cv::Mat &src,
        const cv::Scalar *lb1, const cv::Scalar *ub1,
        const cv::Scalar *lb2=nullptr, const cv::Scalar *ub2=nullptr, 
        unsigned margin=10
    );

    std::vector<std::vector<cv::Point>> findContours(const cv::Mat &src);

    cv::Rect resizeBox(const cv::Rect &r, const unsigned scale);
}

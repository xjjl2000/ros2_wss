#ifndef UTILS_DEF_HPP
#define UTILS_DEF_HPP


#include <cv_bridge/cv_bridge.h>
#include "QImage"


namespace Utils{
        QImage convertToQImage(const cv::Mat &image);
}



#endif
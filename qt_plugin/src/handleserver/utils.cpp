#include "handleserver/utils.hpp"

QImage Utils::convertToQImage(const cv::Mat &image)
{
        cv::Mat rgb;
        if (image.channels() == 3)
        {
                // cvt Mat BGR 2 QImage RGB
                cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
        }
        else
        {
                rgb = image.clone();
        }

        QImage img(rgb.cols, rgb.rows, QImage::Format_RGB888);
        uchar *imageData = img.bits(); // 获取图像数据指针

        // 将像素数据复制到QImage中
        memcpy(imageData, rgb.data, rgb.cols * rgb.rows * 3); // 假设每个像素有三个通道

        return img;
}
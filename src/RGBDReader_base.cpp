#include <RGBDReader/RGBDReader_base.hpp>


void RGBDReader::ICL_NUIM_Reader::readMat(const std::string filename, cv::Mat *img) {
    img->create(height, width, CV_32FC1);
    float x, y, depth_value;
    std::ifstream file;
    file.open(filename.c_str());

    for (size_t i = 0; i < width * height; i++) {
        file >> depth_value; 

        x = i % width;
        y = i / width;
        img->at<float>(y, x) = depth_value;
    }

    cv::normalize(*img, *img, 0, 1, cv::NORM_MINMAX, CV_32F);
}

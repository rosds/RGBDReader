#include <RGBDReader/RGBDReader.h>


void RGBDReader::ICL_NUIM_Reader::readMat(const std::string filename, cv::Mat *img) {
    img->create(height, width, CV_32FC1);
    float x, y, depth_value;
    std::ifstream file;

    try {
        file.open(filename.c_str());
        for (size_t i = 0; i < width * height; i++) {
            file >> depth_value; 
            x = i % width;
            y = i / width;
            img->at<float>(y, x) = depth_value;
        }
        file.close();
    } catch (std::iostream::failure e) {
        std::cerr << "[RGBDReader::readCloud] Failure at reading file ";
        std::cerr << filename;
        std::cerr << ", returning empty matrix." << std::endl;
        img->release();
        return;
    }

    cv::normalize(*img, *img, 0, 1, cv::NORM_MINMAX, CV_32F);
}


void RGBDReader::RGBD_TUM_Reader::readMat(const std::string filename, cv::Mat *img) {
    *img = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);

    if (!img->data) {
        std::cerr << "[RGBDReader::readCloud] Failure at reading file ";
        std::cerr << filename;
        std::cerr << ", returning empty matrix." << std::endl;
        return;
    }

    cv::normalize(*img, *img, 0, 1, cv::NORM_MINMAX, CV_32F);
}

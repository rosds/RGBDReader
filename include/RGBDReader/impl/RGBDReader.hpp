#include <RGBDReader/RGBDReader.h>

template<class OutputCloud>
void RGBDReader::ICL_NUIM_Reader::readCloud(const std::string filename, OutputCloud &cloud) {
    cloud.resize(width * height);
    cloud.width = width;
    cloud.height = height;

    std::vector<float> depth_value(width * height);
    std::ifstream file;
    file.open(filename.c_str());

    for (size_t i = 0; i < width * height; i++) {
        file >> depth_value[i]; 
    }

    #ifdef _OPENMP
    #pragma omp parallel for shared(cloud, depth_value)
    #endif
    for (size_t i = 0; i < width * height; i++) {
        int x = i % width;
        int y = i / width;

        float X = (x - cx) / fx;
        float Y = (y - cy) / -fy;
        float Z = depth_value[i] / sqrt(X * X + Y * Y + 1);

        cloud.points[i].x = X * Z;
        cloud.points[i].y = Y * Z;
        cloud.points[i].z = Z;
    }

    file.close();
}

template<class OutputCloud>
void RGBDReader::RGBD_TUM_Reader::readCloud(const std::string filename, OutputCloud &cloud) {
    cloud.resize(width * height);
    cloud.width = width;
    cloud.height = height;

    std::ifstream file;
    file.open(filename.c_str());

    cv::Mat img = cv::imread(filename, CV_LOAD_IMAGE_ANYDEPTH);
    width = img.cols;
    height = img.rows;

    bool dense = true;

    #ifdef _OPENMP
    #pragma omp parallel for shared(cloud, img)
    #endif
    for (size_t i = 0; i < width * height; i++) {

        float X, Y, Z;
        int x = i % width;
        int y = i / width;

        unsigned short depth = img.at<unsigned short>(i);

        // Handle non depth information
        if (depth == 0) {
            X = Y = Z = std::numeric_limits<float>::quiet_NaN();
            dense = false;
        } else {
            Z = static_cast<float>(depth) / 5000.0f;
            X = (x - cx) * Z / fx;
            Y = (y - cy) * Z / -fy;
        }

        cloud.points[i].x = X;
        cloud.points[i].y = Y;
        cloud.points[i].z = Z;
    }

    cloud.is_dense = dense;

    file.close();
}

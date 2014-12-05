#include <RGBDReader/RGBDReader_base.hpp>


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

        cloud.points[i] = pcl::PointXYZ(X * Z, Y * Z, Z);
    }

    file.close();
}

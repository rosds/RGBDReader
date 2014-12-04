#include <RGBDReader/RGBDReader_base.hpp>


void RGBDReader::ICL_NUIM_Reader::readCloud(const std::string filename, OutputCloud &cloud) {
    // prepare cloud
    cloud.resize(width * height);
    cloud.width = width;
    cloud.height = height;

    float depth_value;
    std::ifstream file;
    file.open(filename.c_str());

    for (size_t y = 0, n = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++, n++) {
            file >> depth_value;

            float X = (x - cx) / fx;
            float Y = (y - cy) / -fy;
            float Z = depth_value / sqrt(X * X + Y * Y + 1);

            cloud.points[n] = pcl::PointXYZ(X * Z, Y * Z, Z);
        }
    }

    file.close();
}

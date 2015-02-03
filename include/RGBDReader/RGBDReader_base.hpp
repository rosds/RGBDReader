#ifndef RGBDREADER_HH
#define RGBDREADER_HH

#include <vector>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>


namespace RGBDReader {

/**
 *  @class Reader
 *
 *  @brief Abstract reader class to handle differente benchmarks readers.
 */
class Reader {};


/**
 *  @brief Reader class for the ICL-NUIM benchmark.
 */
class ICL_NUIM_Reader : public Reader{
public:
    ICL_NUIM_Reader() : 
        width(640), 
        height(480), 
        fx(481.2f), 
        fy(-480.0f), 
        cx(319.5f), 
        cy(239.5f) {}
    ~ICL_NUIM_Reader() {}

    /**
     * @brief Reads the content of the depth file and initializes a
     * pcl::pcl::PointCloud.
     *
     * @param filename [in] Path to a depth text file from the ICL-NUIM dataset.
     * @param cloud [in,out] Resulting cloud.
     */
    template<class OutputCloud>
    void readCloud(const std::string filename, OutputCloud &cloud) {
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

    void readMat(const std::string filename, cv::Mat *img);

private:
    const int width;
    const int height;

    // Camera parameters
    const float fx;
    const float fy;
    const float cx;
    const float cy;
};

} // namespace RGBDReader

#endif // RGBDREADER_HH

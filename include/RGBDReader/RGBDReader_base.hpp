#ifndef RGBDREADER_HH
#define RGBDREADER_HH

#include <vector>
#include <fstream>
#include <string>

#include <pcl/common/common_headers.h>


namespace RGBDReader {

typedef pcl::PointCloud<pcl::PointXYZ> OutputCloud;

/**
 *  @class Reader
 *
 *  @brief Abstract reader class to handle differente benchmarks readers.
 */
class Reader {
public:
    virtual void readCloud(const std::string filename, OutputCloud &cloud) = 0;
};


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
     */
    void readCloud(const std::string filename, OutputCloud &cloud);

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

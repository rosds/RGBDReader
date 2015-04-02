#ifndef RGBDREADER_RGBDREADER_HH
#define RGBDREADER_RGBDREADER_HH

#include <vector>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/common/common_headers.h>


namespace RGBDReader {

/** \class Reader
 *  \brief Abstract reader class to handle differente benchmarks readers.
 */
class Reader {
public:
    int width;
    int height;

    // Camera parameters
    float fx;
    float fy;
    float cx;
    float cy;
};


/** \brief Reader class for the ICL-NUIM benchmark.
 */
class ICL_NUIM_Reader : public Reader{
public:
    ICL_NUIM_Reader() {
        width = 640; 
        height = 480; 
        fx = 481.2f; 
        fy = -480.0f; 
        cx = 319.5f;
        cy = 239.5f;
    }

    /** \brief Reads the content of the depth file and initializes a pcl::PointCloud.
     * \param filename[in] Path to a depth text file from the ICL-NUIM dataset.
     * \param cloud[in,out] Resulting cloud.
     */
    template<class OutputCloud>
    void readCloud(const std::string filename, OutputCloud &cloud);

    void readMat(const std::string filename, cv::Mat *img);
};


class RGBD_TUM_Reader : public Reader{
public:

    RGBD_TUM_Reader() { 
        width = 640;
        height = 480; 
        fx = 568.2044096400148;
        fy = 567.6742468263647;
        cx = 322.9273157602054;
        cy = 240.32862519022834;
    }

    /** \brief Reads the content of the depth file and initializes a pcl::PointCloud.
     * \param filename[in] Path to a depth text file from the ICL-NUIM dataset.
     * \param cloud[in,out] Resulting cloud.
     */
    template<class OutputCloud>
    void readCloud(const std::string filename, OutputCloud &cloud);

    void readMat(const std::string filename, cv::Mat *img);
};

} // namespace RGBDReader

#include <RGBDReader/impl/RGBDReader.hpp>

#endif // RGBDREADER_HH

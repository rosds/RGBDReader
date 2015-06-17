#include <vector>
#include <iostream>
#include <fstream>

#define  EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/program_options.hpp>

#include <RGBDReader/RGBDReader.h>

namespace po = boost::program_options;


int main(int argc, const char *argv[]) {
    cv::Mat img;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());

    RGBDReader::ICL_NUIM_Reader::Ptr icl_reader(new RGBDReader::ICL_NUIM_Reader());
    RGBDReader::RGBD_TUM_Reader::Ptr tum_reader(new RGBDReader::RGBD_TUM_Reader());

    // Parse arguments
    po::options_description desc;
    desc.add_options()
        ("help", "Show help message")
        ("show_img", "Show depth 2D image")
        ("tum_image", po::value<std::string>(), "TUM RGB-D input file")
        ("icl_image", po::value<std::string>(), "ICL NUIM input file");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") || argc < 2) {
        std::cout << desc;
        return 1;
    }

    if (vm.count("tum_image")) {
        std::string filename = vm["tum_image"].as <std::string>();
        std::cout << "Opening file: " << filename << std::endl;

        if (vm.count("show_img")) {
            tum_reader->readMat(filename, &img);

            cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Depth Image", img);

            cv::waitKey(0);
            return 0;
        } else {
            tum_reader->readCloud(filename, *cloud);
            viewer->addPointCloud(cloud);

            while (!viewer->wasStopped()) {
                viewer->spinOnce (100);
            }
        }
    }

    if (vm.count("icl_image")) {
        std::string filename = vm["icl_image"].as <std::string>();
        std::cout << "Opening file: " << filename << std::endl;

        if (vm.count("show_img")) {
            icl_reader->readMat(filename, &img);

            cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Depth Image", img);

            cv::waitKey(0);
            return 0;
        } else {
            icl_reader->readCloud(filename, *cloud);
            viewer->addPointCloud(cloud);

            while (!viewer->wasStopped()) {
                viewer->spinOnce (100);
            }
        }
    }

    return 0;
}

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

#include <RGBDReader/RGBDReader_base.hpp>


int main(int argc, const char *argv[]) {
    cv::Mat img;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    RGBDReader::ICL_NUIM_Reader icl_reader;
    RGBDReader::RGBD_TUM_Reader tum_reader;

    // Parse arguments
    boost::program_options::options_description desc;
    desc.add_options()
        ("help", "Show help message")
        ("show_img", "Show depth image")
        ("tum", "Read from TUM dataset")
        ("frame", boost::program_options::value< std::vector<std::string> >(), "Frame to display");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help") || argc < 2) {
        std::cout << desc;
        return 1;
    }

    if (vm.count("frame")) {
        std::vector<std::string> paths = vm["frame"].as <std::vector<std::string> >();
        std::cout << "Opening file: " << paths[0] << std::endl;

        if (vm.count("show_img")) {
            if (vm.count("tum")) {
                tum_reader.readMat(paths[0], &img);
            } else {
                icl_reader.readMat(paths[0], &img);
            }

            cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Depth Image", img);

            cv::waitKey(0);
            return 0;
        } else {
            if (vm.count("tum")) {
                tum_reader.readCloud(paths[0], *cloud);
            } else {
                icl_reader.readCloud(paths[0], *cloud);
            }
        }
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addPointCloud(cloud);

    while (!viewer->wasStopped()) {
        viewer->spinOnce (100);
    }

    return 0;
}

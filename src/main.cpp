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


double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}

int main(int argc, const char *argv[]) {
    cv::Mat img;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    RGBDReader::ICL_NUIM_Reader reader;

    // Parse arguments
    boost::program_options::options_description desc;
    desc.add_options()
        ("help", "Show help message")
        ("show_img", "Show depth image")
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
            reader.readMat(paths[0], &img);

            cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
            cv::imshow("Depth Image", img);

            cv::waitKey(0);
            return 0;
        } else {
            reader.readCloud(paths[0], *cloud);
        }
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addPointCloud(cloud);

    while (!viewer->wasStopped()) {
        viewer->spinOnce (100);
    }

    return 0;
}

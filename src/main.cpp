#include <vector>
#include <iostream>
#include <fstream>

#define  EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    RGBDReader::ICL_NUIM_Reader reader;

    // Parse arguments
    boost::program_options::options_description desc;
    desc.add_options()
        ("help", "Show help message")
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
        reader.readCloud(paths[0], *cloud);
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->addPointCloud(cloud);

/*
 *    double iss_gamma_21_ (0.999);
 *    double iss_gamma_32_ (0.999);
 *    double iss_min_neighbors_ (25);
 *    int iss_threads_ (4);
 *
 *
 *    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
 *    pcl::search::KdTree<pcl::PointXYZ>::Ptr iss_tree(new pcl::search::KdTree<pcl::PointXYZ>());
 *    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ>::Ptr iss_detector(new pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ>());
 *
 *    iss_detector->setSearchMethod(iss_tree);
 *    iss_detector->setThreshold21 (iss_gamma_21_);
 *    iss_detector->setThreshold32 (iss_gamma_32_);
 *    iss_detector->setMinNeighbors (iss_min_neighbors_);
 *    iss_detector->setNumberOfThreads (iss_threads_);
 *
 *    float model_res = computeCloudResolution(cloud);
 *
 *    iss_detector->setSalientRadius(10 * model_res);
 *    iss_detector->setNonMaxRadius(8 * model_res);
 *
 *    iss_detector->setInputCloud(cloud);
 *    iss_detector->compute(*keypoints);
 *
 *    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(keypoints, 255, 0, 0);
 *
 *    viewer->addPointCloud<pcl::PointXYZ>(keypoints, single_color1, "keypoints");
 *    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints");
 */


    while (!viewer->wasStopped()) {
        viewer->spinOnce (100);
    }

    return 0;
}

#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>

#include <RGBDReader/RGBDReader.h>

namespace po = boost::program_options;

int main(int argc, char *argv[]) {

    // Parse arguments
    po::options_description desc;
    desc.add_options()
        ("input,i", po::value<std::string>()->required(), "Input file")
        ("output,o", po::value<std::string>()->required(), "Output file");

    po::variables_map vm;

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (po::error e) {
        std::cout << desc;
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    RGBDReader::RGBD_TUM_Reader reader;
    reader.readCloud(vm["input"].as<std::string>(), *cloud); 

    std::cout << "Input cloud size: " << cloud->size() << std::endl;
    std::cout << "Writing to file: " << vm["output"].as<std::string>() << std::endl;

    pcl::io::savePCDFileASCII (vm["output"].as<std::string>(), *cloud);

    return 0;
}

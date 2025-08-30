#include <pcl/io/pcd_io.h>
#include <iostream>
#include "pcd_processor.h"
#include "pcd_reader.h"
#include "point_struct_generator.h"


bool PCDProcessor::process(const std::string& input_file, 
                          const std::string& output_struct_file,
                          bool convert_to_ascii) {
    PCDReader reader;
    pcl::PCLPointCloud2 cloud;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version;
    int data_type;
    unsigned int data_idx;

    // Read PCD file header
    if (!reader.readHeader(input_file, cloud, origin, orientation, version, data_type, data_idx)) {
        std::cerr << "Error reading PCD header." << std::endl;
        return false;
    }

    // Print header information
    reader.printHeaderInfo(cloud, data_type);

    // Read full point cloud data
    if (!reader.readData(input_file, cloud)) {
        std::cerr << "Error reading PCD data." << std::endl;
        return false;
    }

    // Check if data is empty
    if (cloud.data.empty()) {
        std::cerr << "Error: Point cloud data is empty." << std::endl;
        return false;
    }

    // Extract field information
    std::vector<FieldInfo> fieldInfo = reader.extractFieldInfo(cloud);

    std::cout << "Data Fields:" << std::endl;
    for (const auto& info : fieldInfo) {
        std::cout << "Name: " << info.name 
                  << ", Size: " << info.size << " bytes"
                  << ", Type: " << info.type 
                  << ", Count: " << info.count 
                  << ", Offset: " << info.offset << std::endl;
    }

    // Generate point structure definition
    PointStructGenerator generator;
    if (!generator.generate(fieldInfo, output_struct_file)) {
        std::cerr << "Error generating point structure." << std::endl;
        return false;
    }

    // Convert to ASCII format if requested and needed
    if (convert_to_ascii && data_type != 0) {
        std::string output_filename = input_file + "_ascii.pcd";
        pcl::PCDWriter writer;
        if (writer.writeASCII(output_filename, cloud, origin, orientation) < 0) {
            std::cerr << "Error saving PCD file as ASCII." << std::endl;
            return false;
        }
        std::cout << "\nSuccessfully converted to ASCII format: " << output_filename << std::endl;
    } else if (data_type == 0) {
        std::cout << "The file is already in ASCII format. No conversion needed." << std::endl;
    }

    return true;
}

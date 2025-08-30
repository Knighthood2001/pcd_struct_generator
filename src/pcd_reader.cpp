#include <pcl/io/pcd_io.h>
#include <iostream>
#include "pcd_reader.h"
#include "type_utils.h"

bool PCDReader::readData(const std::string& filename, pcl::PCLPointCloud2& cloud) {
    return pcl::io::loadPCDFile(filename, cloud) >= 0;
}

std::vector<FieldInfo> PCDReader::extractFieldInfo(const pcl::PCLPointCloud2& cloud) {
    std::vector<FieldInfo> fields;
    for (const auto& field : cloud.fields) {
        if (field.name[0] == '_') continue;
        size_t field_size = 0;
        switch (field.datatype) {
            case 1: case 2: field_size = 1; break;
            case 3: case 4: field_size = 2; break;
            case 5: case 6: case 7: field_size = 4; break;
            case 8: field_size = 8; break;
            default: field_size = 0;
        }
        fields.push_back({field.name, field_size * field.count, 
                         getDataTypeName(field.datatype), field.count, field.offset});
    }
    return fields;
}

void PCDReader::printHeaderInfo(const pcl::PCLPointCloud2& cloud, int data_type) {
    std::cout << "=== PCD File Header Information ===" << std::endl;
    std::cout << "Width: " << cloud.width << std::endl;
    std::cout << "Height: " << cloud.height << std::endl;
    std::cout << "Total Points: " << cloud.width * cloud.height << std::endl;
    std::cout << "Data type: ";
    if (data_type == 0) {
        std::cout << "ASCII" << std::endl;
    } else if (data_type == 1) {
        std::cout << "BINARY" << std::endl;
    } else if (data_type == 2) {
        std::cout << "BINARY_COMPRESSED" << std::endl;
    }
}

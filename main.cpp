#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <fstream>

struct FieldInfo {
    std::string name;
    size_t size;
    std::string type;
    size_t count;
    size_t offset;
};

std::string getDataTypeName(int datatype) {
    switch(datatype) {
        case 1: return "INT8";
        case 2: return "UINT8";
        case 3: return "INT16";
        case 4: return "UINT16";
        case 5: return "INT32";
        case 6: return "UINT32";
        case 7: return "FLOAT32";
        case 8: return "FLOAT64";
        default: return "UNKNOWN";
    }
}

std::string getCppType(const std::string& pclType) {
    if (pclType == "FLOAT32") return "float";
    if (pclType == "FLOAT64") return "double";
    if (pclType == "UINT8") return "std::uint8_t";
    if (pclType == "UINT16") return "std::uint16_t";
    if (pclType == "UINT32") return "std::uint32_t";
    if (pclType == "INT8") return "std::int8_t";
    if (pclType == "INT16") return "std::int16_t";
    if (pclType == "INT32") return "std::int32_t";
    return "unknown_type";
}

std::vector<FieldInfo> extractFieldInfo(const pcl::PCLPointCloud2& cloud) {
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
        fields.push_back({field.name, field_size * field.count, getDataTypeName(field.datatype), field.count, field.offset});
    }
    return fields;
}

void generatePointStruct(const std::vector<FieldInfo>& fieldInfo, const std::string& filename) {
    std::ofstream outFile(filename);
    
    if (!outFile.is_open()) {
        std::cerr << "Error creating file: " << filename << std::endl;
        return;
    }
    
    outFile << "#ifndef POINTLS_TYPES_H\n";
    outFile << "#define POINTLS_TYPES_H\n\n";
    outFile << "#define PCL_NO_PRECOMPILE\n";
    outFile << "#include <pcl/pcl_macros.h>\n";
    outFile << "#include <pcl/point_cloud.h>\n";
    outFile << "#include <pcl/io/pcd_io.h>\n";
    outFile << "#include <pcl/point_types.h>\n";
    outFile << "#include <cstdint>  // For fixed-width integer types\n\n";
    outFile << "struct EIGEN_ALIGN16 PointLS {\n";
    
    // Add struct members
    for (const auto& field : fieldInfo) {
        std::string cppType = getCppType(field.type);
        outFile << "    " << cppType << " " << field.name << ";\n";
    }
    
    outFile << "    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // For Eigen alignment\n";
    outFile << "};\n\n";
    
    outFile << "POINT_CLOUD_REGISTER_POINT_STRUCT (PointLS,\n";
    
    // Add PCL point registration
    for (size_t i = 0; i < fieldInfo.size(); ++i) {
        const auto& field = fieldInfo[i];
        std::string cppType = getCppType(field.type);
        
        outFile << "                                   (" << cppType << ", " << field.name << ", " << field.name << ")";
        if (i < fieldInfo.size() - 1) {
            outFile << "\n";
        } else {
            outFile << ")\n";
        }
    }
    
    outFile << "#endif // POINTLS_TYPES_H\n";
    
    outFile.close();
    std::cout << "Generated point cloud structure in: " << filename << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <pcd_file>" << std::endl;
        return -1;
    }

    pcl::PCDReader reader;
    pcl::PCLPointCloud2 cloud;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version;
    int data_type;
    unsigned int data_idx;

    // 读取PCD文件头部信息
    if (reader.readHeader(argv[1], cloud, origin, orientation, version, data_type, data_idx) < 0) {
        std::cerr << "Error reading PCD header." << std::endl;
        return -1;
    }

    // 输出头部信息
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

    // 读取完整的点云数据（包括实际数据）
    if (reader.read(argv[1], cloud) < 0) {
        std::cerr << "Error reading PCD data." << std::endl;
        return -1;
    }

    // 检查数据是否为空
    if (cloud.data.empty()) {
        std::cerr << "Error: Point cloud data is empty." << std::endl;
        return -1;
    }

    std::vector<FieldInfo> fieldInfo = extractFieldInfo(cloud);

    std::cout << "Data Fields:" << std::endl;
    for (const auto& info : fieldInfo) {
        std::cout << "Name: " << info.name 
                  << ", Size: " << info.size << " bytes"
                  << ", Type: " << info.type 
                  << ", Count: " << info.count 
                  << ", Offset: " << info.offset << std::endl;
    }

    // 生成对应的点云结构体定义
    generatePointStruct(fieldInfo, "pointls_types.h");
    return 0;
}
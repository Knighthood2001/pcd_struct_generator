#include <fstream>
#include <iostream>
#include "point_struct_generator.h"
#include "type_utils.h"


bool PointStructGenerator::generate(const std::vector<FieldInfo>& fieldInfo, 
                                   const std::string& filename) {
    std::ofstream outFile(filename);
    
    if (!outFile.is_open()) {
        std::cerr << "Error creating file: " << filename << std::endl;
        return false;
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
        std::string cppType = getCppType(field.type, field.count);
        outFile << "    " << cppType << " " << field.name << ";\n";
    }
    
    outFile << "    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // For Eigen alignment\n";
    outFile << "};\n\n";
    
    outFile << "POINT_CLOUD_REGISTER_POINT_STRUCT (PointLS,\n";
    
    // Add PCL point registration
    for (size_t i = 0; i < fieldInfo.size(); ++i) {
        const auto& field = fieldInfo[i];
        std::string cppType = getCppType(field.type, field.count);
        
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
    return true;
}

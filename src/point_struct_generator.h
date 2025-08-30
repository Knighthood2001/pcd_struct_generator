#ifndef PCD_PROCESSOR_POINT_STRUCT_GENERATOR_H
#define PCD_PROCESSOR_POINT_STRUCT_GENERATOR_H

#include <vector>
#include <string>
#include "field_info.h"
// 负责生成点云结构体文件，例如：pointls_types.h
class PointStructGenerator {
public:
    bool generate(const std::vector<FieldInfo>& fieldInfo, 
                 const std::string& filename = "pointls_types.h");
};

#endif // PCD_PROCESSOR_POINT_STRUCT_GENERATOR_H
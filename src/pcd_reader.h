#ifndef PCD_PROCESSOR_PCD_READER_H
#define PCD_PROCESSOR_PCD_READER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include "field_info.h"
// 负责读取PCD文件，提取字段信息，并打印头部信息的类。
class PCDReader {
public:
    bool readData(const std::string& filename, pcl::PCLPointCloud2& cloud);
    
    std::vector<FieldInfo> extractFieldInfo(const pcl::PCLPointCloud2& cloud);
    
    void printHeaderInfo(const pcl::PCLPointCloud2& cloud, int data_type);
};


#endif // PCD_PROCESSOR_PCD_READER_H
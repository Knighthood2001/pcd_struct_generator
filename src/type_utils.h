#ifndef PCD_PROCESSOR_TYPE_UTILS_H
#define PCD_PROCESSOR_TYPE_UTILS_H

#include <string>
#include "field_info.h"

std::string getDataTypeName(int datatype);
std::string getCppType(const std::string& pclType, size_t count);


#endif // PCD_PROCESSOR_TYPE_UTILS_H
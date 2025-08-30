#ifndef PCD_PROCESSOR_FIELD_INFO_H
#define PCD_PROCESSOR_FIELD_INFO_H

#include <string>
#include <vector>
#include <iostream>

struct FieldInfo {
    std::string name;
    size_t size;
    std::string type;
    size_t count;
    size_t offset;
};

#endif // PCD_PROCESSOR_FIELD_INFO_H
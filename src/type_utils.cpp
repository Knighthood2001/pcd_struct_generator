#include "type_utils.h"

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

std::string getCppType(const std::string& pclType, size_t count) {
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

#ifndef PCD_PROCESSOR_H
#define PCD_PROCESSOR_H

#include <string>

class PCDProcessor {
public:
    bool process(const std::string& input_file, 
                const std::string& output_struct_file = "pointls_types.h",
                bool convert_to_ascii = true);
};


#endif // PCD_PROCESSOR_H
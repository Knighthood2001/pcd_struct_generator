#include "pcd_processor.h"
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4) {
        std::cerr << "Usage: " << argv[0] << " <pcd_file> [output_struct_file] [convert_to_ascii(0/1)]" << std::endl;
        std::cerr << "Example: " << argv[0] << " input.pcd pointls_types.h 1" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_struct_file = (argc > 2) ? argv[2] : "pointls_types.h";
    bool convert_to_ascii = (argc > 3) ? std::stoi(argv[3]) != 0 : true;

    pcd_processor::PCDProcessor processor;
    if (!processor.process(input_file, output_struct_file, convert_to_ascii)) {
        std::cerr << "Failed to process PCD file." << std::endl;
        return -1;
    }

    return 0;
}
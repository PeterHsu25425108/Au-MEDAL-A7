// Circuit: the parsed device lists and external pin set.
#pragma once
#include <set>
#include <string>
#include <vector>
#include "models.hpp"

namespace aumedal {

class Circuit {
public:
    std::string cell_name;
    std::vector<Mosfet> nfets;
    std::vector<Mosfet> pfets;
    std::set<std::string> ext_pins;
    // Columns grouped by the optional "row" field (CFET DR); row 0 when absent.
    // The stock flow uses nfets/pfets above and ignores these.
    std::vector<std::vector<Mosfet>> row_nfets, row_pfets;

    void read_placement(const std::string& json_path);
};

}

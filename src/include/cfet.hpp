// CFET (imec A7 3.5T, DR template) routing path.
//
// Selected by "technology": "cfet_a7" in design_specs. The stock ASAP7 path
// (fins, Active, GCut, LISD/SDT) is not used at all: tracks, pin points,
// graph pruning, power taps and the GDS are built here from the "cfet"
// section of the config. The SMT core (smt*.cpp) is shared unchanged.
//
// Geometry: x = column index * column_pitch (gate columns even, S/D columns
// odd, a dummy gate on both cell edges); y in [0, rows * row_height], row B
// is row A mirrored about y = row_height. Per column the nFET is the top
// device (S/D contact MD) and the pFET the bottom device (S/D contact BMD);
// both share the gate. All coordinates are config units (unit_nm each).
#pragma once
#include <string>
#include <vector>
#include "circuit.hpp"
#include "config.hpp"
#include "graph.hpp"
#include "models.hpp"
#include "placement.hpp"

namespace aumedal {

struct RoutingResult;

bool is_cfet(const Config& cfg);

// Per-row net orders (index = column), all rows padded to the same length.
struct CfetRows {
    std::vector<NetOrders> rows;
    long ncols = 0;   // number of columns (odd: gate, S/D, ..., gate)
};
CfetRows cfet_rows(const Config& cfg, const Circuit& circ);

struct CfetTracks {
    std::vector<std::vector<long>> x, y, ext_pin_y;   // per routing layer of cfg
};
CfetTracks cfet_tracks(const Config& cfg, const CfetRows& rows);

std::vector<Net> cfet_net_points(const Config& cfg, const Circuit& circ, const CfetRows& rows);

// Drop the edges the generic builder creates but the CFET stack does not have
// (MOL nodes outside their columns, gates across rows, ...); nodes follow.
void cfet_prune_graph(const Config& cfg, const CfetRows& rows, const std::vector<Net>& nets,
                      RoutingGraph& g);

// The CFET counterpart of add_ext_pin_points: every external net must reach an
// M0 signal-track point within its (tolerance-widened) x range.
void cfet_add_ext_pin_points(std::vector<Net>& nets, const Config& cfg,
                             const std::vector<std::vector<long>>& x_points, long tolerance);

void cfet_write_gds(const std::string& path, const std::string& cell_name, const Config& cfg,
                    const RoutingResult& res, const CfetRows& rows);

}

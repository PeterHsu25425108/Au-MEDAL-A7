// CFET (imec A7 3.5T DR) tracks, pin points, graph pruning, power taps and GDS.
// See include/cfet.hpp for the model; numbers come from the config's "cfet"
// section (CFET_DoubleRow/tools/a7_aumedal_config.py writes it from the spec).
#include "cfet.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include "solve.hpp"

namespace aumedal {

namespace {

using Arr = std::array<long, 3>;
using Rect = std::array<double, 4>;

const nlohmann::json& C(const Config& cfg) { return cfg.specs.at("cfet"); }
long cl(const Config& cfg, const char* k) { return C(cfg).at(k).get<long>(); }
std::vector<long> cv(const Config& cfg, const char* k) { return C(cfg).at(k).get<std::vector<long>>(); }

bool is_dummy(const std::string& n) { return n == DUMMY_NET; }
bool is_vdd(const Config& cfg, const std::string& n) { return n == cfg.power_net; }
bool is_vss(const Config& cfg, const std::string& n) { return n == cfg.ground_net; }
bool is_supply(const Config& cfg, const std::string& n) { return is_vdd(cfg, n) || is_vss(cfg, n); }

long nrows(const Config& cfg) { return cl(cfg, "rows"); }
long row_h(const Config& cfg) { return cl(cfg, "row_height"); }
long cell_h(const Config& cfg) { return nrows(cfg) * row_h(cfg); }
// Row r's copy of a row-A y coordinate (odd rows are mirrored).
long ry(const Config& cfg, long r, long y_a) {
    const long base = (r / 2) * 2 * row_h(cfg);
    return (r % 2) ? base + 2 * row_h(cfg) - y_a : base + y_a;
}
std::vector<long> rys(const Config& cfg, long r, const std::vector<long>& ya) {
    std::vector<long> out;
    for (long y : ya) out.push_back(ry(cfg, r, y));
    std::sort(out.begin(), out.end());
    return out;
}
// The centreline (MRW / middle M0 track) between row r and its mirror partner.
long centre_y(const Config& cfg, long r) { return (r / 2) * 2 * row_h(cfg) + row_h(cfg); }

int zi(const Config& cfg, const char* layer) { return cfg.routing_layer_index(layer); }
long colx(const Config& cfg, long xi) { return xi * cl(cfg, "column_pitch"); }

std::vector<long> all_m0_signal(const Config& cfg) {
    std::vector<long> ys;
    for (long r = 0; r < nrows(cfg); ++r)
        for (long y : rys(cfg, r, cv(cfg, "m0_signal_y"))) ys.push_back(y);
    std::sort(ys.begin(), ys.end());
    return ys;
}
std::vector<long> all_centres(const Config& cfg) {
    std::set<long> s;
    for (long r = 0; r < nrows(cfg); ++r) s.insert(centre_y(cfg, r));
    return {s.begin(), s.end()};
}

}  // namespace

bool is_cfet(const Config& cfg) { return cfg.specs.value("technology", std::string()) == "cfet_a7"; }

namespace {
// Column-exact net order of one row: an edge dummy gate, then each device's
// D, G, S with consecutive devices sharing the S/D column (the placement
// lists them abutted), a "dummy" device being one empty gate column, and an
// edge dummy gate. Unlike compute_net_orders nothing is stripped or inserted,
// so the two rows of a DR cell stay column-aligned as placed.
NetOrders row_order(const std::vector<Mosfet>& n, const std::vector<Mosfet>& p) {
    NetOrders no;
    auto push = [&](const std::string& a, const std::string& b) {
        no.n_net.push_back(a); no.p_net.push_back(b); no.n_fins.push_back(0); no.p_fins.push_back(0);
    };
    push(DUMMY_NET, DUMMY_NET);
    for (size_t i = 0; i < n.size(); ++i) {
        const Mosfet& a = n[i];
        const Mosfet& b = p[i];
        if (no.n_net.size() == 1) push(a.drain, b.drain);
        else {
            std::string& ln = no.n_net.back();
            std::string& lp = no.p_net.back();
            auto fits = [](const std::string& last, const std::string& d) {
                return last == d || last == DUMMY_NET || d == DUMMY_NET;
            };
            if (!fits(ln, a.drain) || !fits(lp, b.drain))
                throw std::runtime_error("cfet placement: column " + std::to_string(i) +
                                         " does not abut the previous one (" + ln + "/" + lp + " vs " +
                                         a.drain + "/" + b.drain + "); insert a dummy column");
            if (a.drain != DUMMY_NET) ln = a.drain;
            if (b.drain != DUMMY_NET) lp = b.drain;
        }
        push(a.gate, b.gate);
        push(a.source, b.source);
    }
    push(DUMMY_NET, DUMMY_NET);
    return no;
}
}  // namespace

CfetRows cfet_rows(const Config& cfg, const Circuit& circ) {
    CfetRows out;
    const long R = nrows(cfg);
    if ((long)circ.row_nfets.size() > R)
        throw std::runtime_error("placement uses more rows than the config's cfet.rows");
    for (long r = 0; r < R; ++r) {
        NetOrders no;
        if (r < (long)circ.row_nfets.size() && !circ.row_nfets[r].empty())
            no = row_order(circ.row_nfets[r], circ.row_pfets[r]);
        out.rows.push_back(no);
        out.ncols = std::max(out.ncols, (long)no.n_net.size());
    }
    if (out.ncols == 0) throw std::runtime_error("placement has no device");
    for (auto& no : out.rows) {
        if (no.n_net.empty()) { no.n_net = {DUMMY_NET}; no.p_net = {DUMMY_NET}; no.n_fins = {0}; no.p_fins = {0}; }
        while ((long)no.n_net.size() < out.ncols) {
            no.n_net.push_back(DUMMY_NET); no.p_net.push_back(DUMMY_NET);
            no.n_fins.push_back(0); no.p_fins.push_back(0);
        }
    }
    return out;
}

CfetTracks cfet_tracks(const Config& cfg, const CfetRows& rows) {
    CfetTracks t;
    const int L = (int)cfg.routing_layers.size();
    t.x.assign(L, {}); t.y.assign(L, {}); t.ext_pin_y.assign(L, {});
    // Interior columns only: the cell-edge columns are shared with the neighbour.
    std::vector<long> xs;
    for (long xi = 1; xi + 1 < rows.ncols; ++xi) xs.push_back(colx(cfg, xi));

    std::vector<long> m0 = all_m0_signal(cfg);
    for (long c : all_centres(cfg)) m0.push_back(c);
    std::sort(m0.begin(), m0.end());

    std::vector<long> bm0, bmd;
    for (long r = 0; r < nrows(cfg); ++r)
        for (long y : rys(cfg, r, cv(cfg, "bm0_signal_y"))) bm0.push_back(y);
    std::sort(bm0.begin(), bm0.end());
    bmd = bm0;
    for (long c : all_centres(cfg)) bmd.push_back(c);
    std::sort(bmd.begin(), bmd.end());

    std::vector<long> m2 = cv(cfg, "m2_y");
    std::set<long> m1s(m0.begin(), m0.end());
    m1s.insert(m2.begin(), m2.end());
    std::vector<long> m1(m1s.begin(), m1s.end());

    for (int z = 0; z < L; ++z) {
        const std::string& l = cfg.routing_layers[z];
        t.x[z] = xs;
        if (l == "BM0") t.y[z] = bm0;
        else if (l == "BMD") t.y[z] = bmd;
        else if (l == "MD" || l == "GATE" || l == "M0") t.y[z] = m0;
        else if (l == "M1") t.y[z] = m1;
        else if (l == "M2") t.y[z] = m2;
        else throw std::runtime_error("cfet: unknown routing layer " + l);
    }
    return t;
}

namespace {

struct NetBuilder {
    const Config& cfg;
    const Circuit& circ;
    std::map<std::string, Net> nets;
    std::vector<std::string> order;

    Net& net(const std::string& name) {
        auto it = nets.find(name);
        if (it == nets.end()) {
            Net n;
            n.name = name;
            n.is_power = false;
            n.is_ext_pin = circ.ext_pins.count(name) > 0;
            order.push_back(name);
            it = nets.emplace(name, std::move(n)).first;
        }
        return it->second;
    }
    void pin(const std::string& name, long term, long x, const std::vector<long>& ys, int z) {
        Pin p; p.term = term;
        for (long y : ys) p.points.push_back({x, y, z});
        net(name).pins.push_back(std::move(p));
    }
};

}  // namespace

std::vector<Net> cfet_net_points(const Config& cfg, const Circuit& circ, const CfetRows& rows) {
    NetBuilder b{cfg, circ, {}, {}};
    const int zg = zi(cfg, "GATE"), zmd = zi(cfg, "MD"), zbmd = zi(cfg, "BMD");
    for (long r = 0; r < nrows(cfg); ++r) {
        const NetOrders& no = rows.rows[r];
        const std::vector<long> sig = rys(cfg, r, cv(cfg, "m0_signal_y"));
        const std::vector<long> bsig = rys(cfg, r, cv(cfg, "bm0_signal_y"));
        for (long xi = 0; xi < rows.ncols; ++xi) {
            const std::string& n = no.n_net[xi];
            const std::string& p = no.p_net[xi];
            const long x = colx(cfg, xi);
            if (xi % 2 == 0) {                       // gate column
                if (is_dummy(n) && is_dummy(p)) continue;
                if (!is_dummy(n) && !is_dummy(p) && n != p)
                    throw std::runtime_error("cfet: n/p gate nets differ in a column (" + n + " / " + p + ")");
                const std::string g = is_dummy(n) ? p : n;
                if (is_supply(cfg, g)) throw std::runtime_error("cfet: gate tied to a supply is not supported (" + g + ")");
                b.pin(g, TERM_GATE, x, sig, zg);
            } else {                                 // S/D column
                if (!is_dummy(n)) {
                    if (is_vdd(cfg, n)) throw std::runtime_error("cfet: nFET S/D on " + n + " is not supported");
                    if (!is_vss(cfg, n)) b.pin(n, TERM_SD, x, sig, zmd);
                }
                if (!is_dummy(p)) {
                    if (is_vss(cfg, p)) throw std::runtime_error("cfet: pFET S/D on " + p + " is not supported");
                    if (!is_vdd(cfg, p)) b.pin(p, TERM_SD, x, bsig, zbmd);
                }
            }
        }
    }
    std::vector<Net> out;
    for (const auto& name : b.order) out.push_back(b.nets.at(name));
    return out;
}

void cfet_prune_graph(const Config& cfg, const CfetRows& rows, const std::vector<Net>&,
                      RoutingGraph& g) {
    const int zg = zi(cfg, "GATE"), zmd = zi(cfg, "MD"), zbmd = zi(cfg, "BMD");
    // Which rows (by y) a coordinate belongs to; centreline belongs to both of its rows.
    auto row_of = [&](long y) { return std::min(nrows(cfg) - 1, y / row_h(cfg)); };
    // Allowed MOL nodes.
    std::set<Arr> ok;
    for (long r = 0; r < nrows(cfg); ++r) {
        const NetOrders& no = rows.rows[r];
        const std::vector<long> sig = rys(cfg, r, cv(cfg, "m0_signal_y"));
        const std::vector<long> bsig = rys(cfg, r, cv(cfg, "bm0_signal_y"));
        const long c = centre_y(cfg, r);
        for (long xi = 1; xi + 1 < rows.ncols; ++xi) {
            const long x = colx(cfg, xi);
            const std::string& n = no.n_net[xi];
            const std::string& p = no.p_net[xi];
            if (xi % 2 == 0) {
                if (is_dummy(n) && is_dummy(p)) continue;
                for (long y : sig) ok.insert({x, y, zg});
            } else {
                const bool n_sig = !is_dummy(n) && !is_supply(cfg, n);
                const bool p_sig = !is_dummy(p) && !is_supply(cfg, p);
                if (n_sig) { for (long y : sig) ok.insert({x, y, zmd}); ok.insert({x, c, zmd}); }
                if (p_sig) {
                    for (long y : bsig) ok.insert({x, y, zbmd});
                    ok.insert({x, c, zbmd});
                    ok.insert({x, c, zmd});          // MRW top, reachable through the MRW
                }
            }
        }
    }
    auto mol = [&](const Arr& a) { return a[2] == zg || a[2] == zmd || a[2] == zbmd; };
    std::set<std::pair<Arr, Arr>> kept;
    for (const auto& e : g.edges) {
        const Arr& a = e.first; const Arr& b = e.second;
        if (mol(a) && !ok.count(a)) continue;
        if (mol(b) && !ok.count(b)) continue;
        if (a[2] == b[2] && a[2] == zg && row_of(a[1]) != row_of(b[1])) continue;   // gate cut at the centreline
        kept.insert(e);
    }
    g.edges.swap(kept);
    g.nodes.clear();
    for (const auto& e : g.edges) { g.nodes.insert(e.first); g.nodes.insert(e.second); }
}

void cfet_add_ext_pin_points(std::vector<Net>& nets, const Config& cfg,
                             const std::vector<std::vector<long>>& x_points, long tolerance) {
    const int z0 = zi(cfg, "M0");
    if (z0 < 0) return;
    const std::vector<long> ys = all_m0_signal(cfg);
    for (Net& n : nets) {
        if (!n.is_ext_pin || n.is_power || n.pins.empty()) continue;
        long mn = std::numeric_limits<long>::max(), mx = std::numeric_limits<long>::min();
        for (const Pin& p : n.pins)
            for (const Point& pt : p.points) { mn = std::min(mn, pt.x); mx = std::max(mx, pt.x); }
        mn -= (long)(tolerance * cfg.x_unit);
        mx += (long)(tolerance * cfg.x_unit);
        Pin ext; ext.term = TERM_EXT_PIN;
        for (long x : x_points[z0])
            if (mn <= x && x <= mx)
                for (long y : ys) ext.points.push_back({x, y, z0});
        if (!ext.points.empty()) n.pins.push_back(std::move(ext));
    }
}

// ------------------------------------------------------------------ GDS
namespace {

void put16(std::vector<uint8_t>& b, uint16_t v) { b.push_back(v >> 8); b.push_back(v & 0xFF); }
void put32(std::vector<uint8_t>& b, int32_t v) {
    uint32_t u = (uint32_t)v;
    b.push_back(u >> 24); b.push_back((u >> 16) & 0xFF); b.push_back((u >> 8) & 0xFF); b.push_back(u & 0xFF);
}
void put_real8(std::vector<uint8_t>& b, double v) {
    if (v == 0.0) { for (int i = 0; i < 8; ++i) b.push_back(0); return; }
    uint64_t sign = 0;
    if (v < 0) { sign = 1; v = -v; }
    int exp = 64;
    while (v >= 1.0) { v /= 16.0; ++exp; }
    while (v < 1.0 / 16.0) { v *= 16.0; --exp; }
    uint64_t mant = (uint64_t)(v * 72057594037927936.0);
    uint64_t r = (sign << 63) | ((uint64_t)(exp & 0x7F) << 56) | (mant & 0x00FFFFFFFFFFFFFFULL);
    for (int i = 7; i >= 0; --i) b.push_back((r >> (i * 8)) & 0xFF);
}
void record(std::ofstream& f, uint16_t token, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> h;
    put16(h, (uint16_t)(4 + data.size()));
    put16(h, token);
    f.write((const char*)h.data(), h.size());
    if (!data.empty()) f.write((const char*)data.data(), data.size());
}
std::vector<uint8_t> str_data(const std::string& s) {
    std::vector<uint8_t> d(s.begin(), s.end());
    if (d.size() & 1) d.push_back(0);
    return d;
}

struct Label { std::string text; double x, y; std::string layer; };

struct Shapes {
    std::map<std::string, std::vector<Rect>> rects;
    std::vector<Label> labels;
    void add(const std::string& l, double x0, double y0, double x1, double y1) {
        rects[l].push_back({std::min(x0, x1), std::min(y0, y1), std::max(x0, x1), std::max(y0, y1)});
    }
};

std::vector<long> via_size(const Config& cfg, const char* v) { return C(cfg).at("via_size").at(v).get<std::vector<long>>(); }

}  // namespace

void cfet_write_gds(const std::string& path, const std::string& cell_name, const Config& cfg,
                    const RoutingResult& res, const CfetRows& rows) {
    Shapes S;
    const long R = nrows(cfg), H = cell_h(cfg), W = colx(cfg, rows.ncols - 1);
    const long enc = cl(cfg, "enclosure");
    const std::vector<long> gy = cv(cfg, "gate_y"), sy = cv(cfg, "sheet_y"), ky = cv(cfg, "contact_y");
    const long gw = cl(cfg, "gate_width"), kw = cl(cfg, "contact_width");
    const long cover = cl(cfg, "mrw_cover_y");                 // row-A y the extended contact reaches
    const std::vector<long> mrw = via_size(cfg, "MRW");
    const auto& layers = cfg.routing_layers;
    auto lname = [&](long z) { return layers[z]; };
    const int zg = zi(cfg, "GATE"), zmd = zi(cfg, "MD"), zbmd = zi(cfg, "BMD"), zm0 = zi(cfg, "M0");
    const int zbm0 = zi(cfg, "BM0");

    // Which contacts/vias the solution uses.
    std::set<std::pair<Arr, Arr>> used(res.metals.begin(), res.metals.end());
    auto has = [&](const Arr& a, const Arr& b) { return used.count(a < b ? std::make_pair(a, b) : std::make_pair(b, a)) > 0; };

    // ---- fixed geometry
    S.add("BOUNDARY", 0, 0, W, H);
    const long m0r = cl(cfg, "m0_rail_half"), bm0r = cl(cfg, "bm0_rail_half");
    for (long k = 0; k <= R; k += 2) {                          // edge rails, clipped at the boundary
        const long y = k * row_h(cfg);
        S.add("M0", 0, std::max(0L, y - m0r), W, std::min(H, y + m0r));
        S.add("BM0", 0, std::max(0L, y - bm0r), W, std::min(H, y + bm0r));
        S.labels.push_back({cfg.ground_net, W / 2.0, y == 0 ? m0r / 2.0 : H - m0r / 2.0, "M0"});
        S.labels.push_back({cfg.power_net, W / 2.0, y == 0 ? bm0r / 2.0 : H - bm0r / 2.0, "BM0"});
    }
    for (long c : all_centres(cfg)) {                          // centreline backside VSS, whole
        S.add("BM0", 0, c - bm0r, W, c + bm0r);
        S.labels.push_back({cfg.ground_net, W / 2.0, (double)c, "BM0"});
    }

    for (long r = 0; r < R; ++r) {
        const NetOrders& no = rows.rows[r];
        bool any = false;
        for (long xi = 0; xi < rows.ncols; ++xi) any |= !is_dummy(no.n_net[xi]) || !is_dummy(no.p_net[xi]);
        // gates
        for (long xi = 0; xi < rows.ncols; xi += 2) {
            const bool edge = (xi == 0 || xi == rows.ncols - 1);
            if (!any && !edge) continue;
            const double x = colx(cfg, xi);
            S.add("GATE", x - gw / 2.0, ry(cfg, r, gy[0]), x + gw / 2.0, ry(cfg, r, gy[1]));
        }
        // sheets: one per run of columns with a device in that tier
        for (int tier = 0; tier < 2; ++tier) {
            const auto& nets = tier ? no.p_net : no.n_net;
            const char* layer = tier ? "PSHEET" : "NSHEET";
            long xi = 1;
            while (xi < rows.ncols) {
                if (xi % 2 == 0 && !is_dummy(nets[xi])) {
                    long a = xi, b = xi;
                    while (b + 2 < rows.ncols && !is_dummy(nets[b + 2])) b += 2;
                    S.add(layer, colx(cfg, a - 1) - kw / 2.0, ry(cfg, r, sy[0]), colx(cfg, b + 1) + kw / 2.0, ry(cfg, r, sy[1]));
                    xi = b + 1;
                }
                ++xi;
            }
        }
        // S/D contacts and power taps
        const long c = centre_y(cfg, r);
        for (long xi = 1; xi < rows.ncols; xi += 2) {
            const long x = colx(cfg, xi);
            for (int tier = 0; tier < 2; ++tier) {
                const std::string& net = tier ? no.p_net[xi] : no.n_net[xi];
                if (is_dummy(net)) continue;
                const char* layer = tier ? "BMD" : "MD";
                const int z = tier ? zbmd : zmd;
                long lo = ky[0], hi = ky[1];                     // row-A coordinates
                if (is_supply(cfg, net)) lo = cl(cfg, tier ? "bmd_tap_y" : "md_tap_y");
                const std::vector<long> pins = tier ? rys(cfg, r, cv(cfg, "bm0_signal_y")) : rys(cfg, r, cv(cfg, "m0_signal_y"));
                bool ext = false;
                for (long y : pins) ext |= has({x, y, z}, {x, c, z});
                if (ext) hi = cover;
                S.add(layer, x - kw / 2.0, ry(cfg, r, lo), x + kw / 2.0, ry(cfg, r, hi));
                if (is_supply(cfg, net)) {
                    const long rail = ry(cfg, r, 0);
                    const std::vector<long> v = via_size(cfg, tier ? "BVD" : "VD");
                    S.add(tier ? "BVD" : "VD", x - v[0] / 2.0, rail - v[1] / 2.0, x + v[0] / 2.0, rail + v[1] / 2.0);
                }
            }
        }
    }

    // ---- MRW: present wherever a contact reaches the centreline (spec 4: a
    // contact extends over the MRW slot only to land on an MRW)
    {
        std::set<std::pair<long, long>> mrw_at;
        std::set<long> cs;
        for (long c : all_centres(cfg)) cs.insert(c);
        for (const auto& e : res.metals)
            for (const Arr* n : {&e.first, &e.second})
                if (((*n)[2] == zmd || (*n)[2] == zbmd) && cs.count((*n)[1])) mrw_at.insert({(*n)[0], (*n)[1]});
        for (const auto& xy : mrw_at)
            S.add("MRW", xy.first - mrw[0] / 2.0, xy.second - mrw[1] / 2.0, xy.first + mrw[0] / 2.0, xy.second + mrw[1] / 2.0);
    }

    // ---- routed wires: merge used in-layer edges into straight segments
    std::map<long, std::vector<std::pair<Arr, Arr>>> by_z;
    std::set<Arr> via_lo, via_hi;   // nodes with a via below / above
    for (const auto& e : res.metals) {
        if (e.first[2] == e.second[2]) by_z[e.first[2]].push_back(e);
        else {
            const Arr& lo = e.first[2] < e.second[2] ? e.first : e.second;
            const Arr& hi = e.first[2] < e.second[2] ? e.second : e.first;
            via_hi.insert(lo); via_lo.insert(hi);
            const std::string a = lname(lo[2]), b = lname(hi[2]);
            const double x = lo[0], y = lo[1];
            std::string v;
            if (a == "BM0" && b == "BMD") v = "BVD";
            else if (a == "BMD" && b == "MD") v = "MRW";
            else if (a == "MD" && b == "M0") v = "VD";
            else if (a == "GATE" && b == "M0") v = "VG";
            else if (a == "M0" && b == "M1") v = "V0";
            else if (a == "M1" && b == "M2") v = "V1";
            else throw std::runtime_error("cfet gds: unexpected via " + a + "-" + b);
            const std::vector<long> s = via_size(cfg, v.c_str());
            if (v != "MRW") S.add(v, x - s[0] / 2.0, y - s[1] / 2.0, x + s[0] / 2.0, y + s[1] / 2.0);
        }
    }
    // Along-wire half length of the via at a node (0 without via).
    auto via_reach = [&](const Arr& n, bool horizontal) -> double {
        double best = 0;
        const std::string l = lname(n[2]);
        auto take = [&](const char* v) {
            const std::vector<long> s = via_size(cfg, v);
            best = std::max(best, (horizontal ? s[0] : s[1]) / 2.0 + enc);
        };
        if (via_lo.count(n)) {
            if (l == "M0") take("VD");                // VG and VD are the same size
            else if (l == "M1") take("V0");
            else if (l == "M2") take("V1");
            else if (l == "BMD") take("BVD");
        }
        if (via_hi.count(n)) {
            if (l == "M0") take("V0");
            else if (l == "M1") take("V1");
            else if (l == "BM0") take("BVD");
        }
        return best;
    };
    const std::map<std::string, long> width = C(cfg).at("wire_width").get<std::map<std::string, long>>();
    const long mid_w = cl(cfg, "m0_middle_width");
    std::set<long> centres;
    for (long c : all_centres(cfg)) centres.insert(c);
    std::set<Arr> wired;   // nodes covered by a drawn wire
    for (auto& kv : by_z) {
        const long z = kv.first;
        const std::string l = lname(z);
        if (z == zg || z == zmd || z == zbmd) continue;            // contacts drawn above
        const int dir = cfg.routing_directions[z];
        const bool horiz = (dir == HORIZONTAL);
        // union-find over nodes on straight runs
        std::map<Arr, std::vector<Arr>> adj;
        for (const auto& e : kv.second) { adj[e.first].push_back(e.second); adj[e.second].push_back(e.first); }
        std::set<Arr> seen;
        for (const auto& st : adj) {
            if (seen.count(st.first)) continue;
            std::vector<Arr> comp, stack{st.first};
            while (!stack.empty()) {
                Arr a = stack.back(); stack.pop_back();
                if (!seen.insert(a).second) continue;
                comp.push_back(a);
                for (const Arr& b : adj[a]) if (!seen.count(b)) stack.push_back(b);
            }
            std::sort(comp.begin(), comp.end(), [&](const Arr& a, const Arr& b) { return horiz ? a[0] < b[0] : a[1] < b[1]; });
            const Arr& a = comp.front(); const Arr& b = comp.back();
            for (const Arr& n : comp) wired.insert(n);
            long w = width.at(l);
            if (l == "M0" && centres.count(a[1])) w = mid_w;
            if (horiz)
                S.add(l, a[0] - via_reach(a, true), a[1] - w / 2.0, b[0] + via_reach(b, true), a[1] + w / 2.0);
            else
                S.add(l, a[0] - w / 2.0, a[1] - via_reach(a, false), a[0] + w / 2.0, b[1] + via_reach(b, false));
        }
    }
    // Landing pads for vias on a routing-layer node that carries no wire (via stacks).
    for (const std::set<Arr>* vs : {&via_lo, &via_hi})
        for (const Arr& n : *vs) {
            const std::string l = lname(n[2]);
            if (wired.count(n) || n[2] == zg || n[2] == zmd || n[2] == zbmd) continue;
            const int dir = cfg.routing_directions[n[2]];
            const bool horiz = (dir == HORIZONTAL);
            long w = width.at(l);
            if (l == "M0" && centres.count(n[1])) w = mid_w;
            const double reach = via_reach(n, horiz);
            if (horiz) S.add(l, n[0] - reach, n[1] - w / 2.0, n[0] + reach, n[1] + w / 2.0);
            else S.add(l, n[0] - w / 2.0, n[1] - reach, n[0] + w / 2.0, n[1] + reach);
        }
    (void)zm0; (void)zbm0;

    // ---- pin labels: one M0 signal-track node per external net
    {
        std::set<long> sig;
        for (long y : all_m0_signal(cfg)) sig.insert(y);
        std::map<std::string, Arr> best;
        for (const auto& ne : res.net_edges) {
            for (const Arr* n : {&ne.second.first, &ne.second.second}) {
                if ((*n)[2] != zi(cfg, "M0") || !sig.count((*n)[1])) continue;
                if (!best.count(ne.first)) best[ne.first] = *n;
            }
        }
        for (const auto& kv : best)
            if (res.ext_nets.count(kv.first)) S.labels.push_back({kv.first, (double)kv.second[0], (double)kv.second[1], "M0"});
    }

    // ---- write
    const auto& lm = cfg.specs.at("layer_map");
    const double unit_nm = C(cfg).at("unit_nm").get<double>();
    const double dbu_nm = cfg.gds_database_unit_nm;
    const double scale = unit_nm / dbu_nm;
    std::ofstream f(path, std::ios::binary);
    { std::vector<uint8_t> d; put16(d, 600); record(f, 0x0002, d); }
    { std::vector<uint8_t> d; for (int i = 0; i < 12; ++i) put16(d, 0); record(f, 0x0102, d); }
    record(f, 0x0206, str_data(cell_name));
    { std::vector<uint8_t> d; put_real8(d, dbu_nm / 1000.0); put_real8(d, dbu_nm * 1e-9); record(f, 0x0305, d); }
    { std::vector<uint8_t> d; for (int i = 0; i < 12; ++i) put16(d, 0); record(f, 0x0502, d); }
    record(f, 0x0606, str_data(cell_name));
    for (const auto& kv : S.rects) {
        const int gl = lm.at(kv.first).get<int>();
        for (const Rect& r : kv.second) {
            record(f, 0x0800, {});
            { std::vector<uint8_t> d; put16(d, (uint16_t)gl); record(f, 0x0D02, d); }
            { std::vector<uint8_t> d; put16(d, 0); record(f, 0x0E02, d); }
            { std::vector<uint8_t> d;
              const int32_t lx = (int32_t)std::lround(r[0] * scale), ly = (int32_t)std::lround(r[1] * scale);
              const int32_t ux = (int32_t)std::lround(r[2] * scale), uy = (int32_t)std::lround(r[3] * scale);
              put32(d, lx); put32(d, ly); put32(d, ux); put32(d, ly); put32(d, ux); put32(d, uy);
              put32(d, lx); put32(d, uy); put32(d, lx); put32(d, ly); record(f, 0x1003, d); }
            record(f, 0x1100, {});
        }
    }
    for (const Label& lb : S.labels) {
        record(f, 0x0C00, {});
        { std::vector<uint8_t> d; put16(d, (uint16_t)lm.at(lb.layer).get<int>()); record(f, 0x0D02, d); }
        { std::vector<uint8_t> d; put16(d, 0); record(f, 0x1602, d); }
        { std::vector<uint8_t> d; put32(d, (int32_t)std::lround(lb.x * scale)); put32(d, (int32_t)std::lround(lb.y * scale)); record(f, 0x1003, d); }
        record(f, 0x1906, str_data(lb.text));
        record(f, 0x1100, {});
    }
    record(f, 0x0700, {});
    record(f, 0x0400, {});
}

}  // namespace aumedal

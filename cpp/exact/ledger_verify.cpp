// Verifier of a ledger record (criterion 6 of the M2 design note, section 11).
// It builds a new ledger from the header of the record and replays every offer,
// branch and close with its stored arguments, recomputing every exact pass and
// every bound. It links the exact library only (exact pass, ledger, the exact
// header of MAMUT-routing-lib and GMP), nothing of the search and no LP backend.
//
// Usage: kayros_ledger_verify RECORD
// Prints "certified U=<exact fraction>" and exits 0 iff the replay certifies,
// else prints "not certified: <reason>" and exits 1 (2 on a usage error).
#include "exact/ledger.h"

#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "usage: kayros_ledger_verify RECORD\n";
        return 2;
    }
    std::ifstream in(argv[1]);
    if (!in) {
        std::cout << "not certified: cannot open " << argv[1] << '\n';
        return 1;
    }
    const kayros::exact::Replay r = kayros::exact::replay(in);
    if (!r.certified) {
        std::cout << "not certified: " << r.message << " (" << r.operations << " operations read)\n";
        return 1;
    }
    std::cout << "certified U=" << r.U->get_str() << " operations=" << r.operations << " routes=" << r.incumbent.size() << '\n';
    for (const auto &route : r.incumbent) {
        std::cout << "route";
        for (int v : route) std::cout << ' ' << v;
        std::cout << '\n';
    }
    return 0;
}

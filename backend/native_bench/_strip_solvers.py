"""Drop specific solver rows from a master-benchmark results CSV, in place, so a
subsequent `run_native_master.py --resume --solvers <those>` re-runs exactly those
cells (with the new native-C++ CCD/FABRIK) and leaves every other row untouched.

Usage:  python3 native_bench/_strip_solvers.py <csv_path> <solver_id> [<solver_id> ...]
Writes a one-time <csv_path>.bak alongside before rewriting.
"""
from __future__ import annotations

import csv
import shutil
import sys

def main() -> int:
    csv_path, drop = sys.argv[1], set(sys.argv[2:])
    if not drop:
        print("no solver ids given", file=sys.stderr)
        return 2
    with open(csv_path, newline="", encoding="utf-8") as f:
        rd = csv.DictReader(f)
        fields = rd.fieldnames
        rows = list(rd)
    kept = [r for r in rows if r["solver"] not in drop]
    dropped = len(rows) - len(kept)
    shutil.copyfile(csv_path, csv_path + ".bak")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(kept)
    print(f"{csv_path}: dropped {dropped} rows for {sorted(drop)}, kept {len(kept)} "
          f"(backup -> {csv_path}.bak)")
    return 0

if __name__ == "__main__":
    sys.exit(main())

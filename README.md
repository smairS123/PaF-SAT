# PaF-SAT

Code for **PaF-SAT**, the SAT instance of the *Plan-and-Flow* framework for multi-agent path finding with unassigned
agents: assigned agents each have a goal, unassigned agents have no goal and only have to move out of the way.
PaF-SAT returns a plan with the minimum makespan.

Paper: *Plan-and-Flow: Makespan-Optimal MAPF with Unassigned Agents via Min-Cut Feedback*,
Smair Smair, Fouad Haddad, Oren Salzman (WoMAPF 2027).

## How it works

* **Ph1: Assigned-Agent Planning.** NRF-SAT plans the assigned agents for a given makespan.
* **Ph2: Unassigned-Agent Flow.** A max-flow test on the time-expanded graph checks whether the unassigned agents can
  move around the assigned agents' plan.
* On failure, Ph2 returns **min-cut feedback**, which PaF-SAT adds to Ph1 as a clause. The clause is built from the
  earliest failed prefix and the minimal realization conflict computed on the merged network. Corridors with slacks 2
  and then 8 are switched on when the full MDD has at least 10^5 nodes.

## Requirements

* C++20 compiler, CMake >= 3.21, Boost (`program_options`)
* [boOX](https://github.com/surynek/boOX) by Pavel Surynek, which holds the NRF-SAT encoder and the Glucose SAT
  solver. It is not redistributed here. Use revision `c27ce6f` and apply the patch below.
* OR-Tools is optional (`-DWITH_ORTOOLS=ON`) and only needed for verification and fallback flow paths.

## Build

```bash
git clone https://github.com/surynek/boOX.git && cd boOX && git checkout c27ce6f
git apply /path/to/PaF-SAT/patches/boox-makespan.patch
(cd boOX-y && bash compile-opt.sh > src/compile.h)   # boOX's optimized configuration (no debug mode)
cd ..

cd PaF-SAT
cmake -S . -B build -DBOOX_DIR=/path/to/boOX
cmake --build build --parallel
```

`BOOX_DIR` is the folder that contains `boOX-y/`. The build itself does not modify the boOX checkout.

## Run

```bash
./build/paf_sat -m map.map -s tasks.scen -t 300 -o results.csv --solution paths.csv
```

| Option | Meaning |
|---|---|
| `-m, --map` | map file (see Input format) |
| `-s, --scenario` | scenario file (see Input format) |
| `-t, --time` | time limit in seconds (default 30) |
| `--max-makespan` | optional horizon cap (default: none) |
| `-o, --output` | append one result row to this CSV |
| `--solution` | write the validated paths as `role,agent,time,x,y` |
| `--trace` | write the learned clauses as CSV |

Every solution is validated before it is reported.

## Input format

**Map.** Either a MovingAI `.map` file, or a text file whose first line is `rows columns`, followed by the grid
(`.` free, `@` blocked).

**Scenario.** Coordinates are `x y` with `x` = column and `y` = row, both from 0. Raw MovingAI `.scen` files are not
read.

```
A U                 number of assigned and unassigned agents
sx sy gx gy         one line per assigned agent: start and goal
...
x y                 one line per unassigned agent: start
...
```

## Environment switches

The final configuration is the default. These switches exist for experiments:

| Variable | Default | Effect |
|---|---|---|
| `UA_CORRIDOR` | on | corridors |
| `UA_CORRIDOR_SLACKS` | `2,8` | corridor slacks |
| `UA_CORRIDOR_MIN_NODES` | `100000` | number of MDD nodes from which corridors are used |
| `UA_KEEP_CUTS`, `UA_KEEP_CLAUSES` | on | keep learned clauses across horizons |

## Acknowledgments

* The flow and constraints code in `lib/` started from the code of `wh-rearrangement` (CRL, Technion) and was changed
  for PaF-SAT.
* Ph1 uses [boOX](https://github.com/surynek/boOX) by Pavel Surynek (NRF-SAT and the Glucose SAT solver), with the
  changes in `patches/boox-makespan.patch`.

## License

To be added.

## Citation

```
@inproceedings{paf-sat,
  title     = {Plan-and-Flow: Makespan-Optimal {MAPF} with Unassigned Agents via Min-Cut Feedback},
  author    = {Smair, Smair and Haddad, Fouad and Salzman, Oren},
  booktitle = {Workshop on Multi-Agent Path Finding (WoMAPF)},
  year      = {2027}
}
```

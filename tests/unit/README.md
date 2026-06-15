# ForRocket C++ Unit Tests

Module-level unit tests (GoogleTest) with gcov/gcovr code coverage. These
complement the end-to-end Python suite in [`../simulation_test.py`](../simulation_test.py):
the Python suite drives the whole built binary, while these tests exercise
individual C++ functions in isolation so failures point straight at a module.

## Coverage status (2026-06-15)

264 tests across 32 `test_*.cpp` files. Whole-`src/` coverage (see "Coverage"
below for the metric): **C0 lines 96.4%, C1 branches 93.2%.** Every module is
≥87% C1 except two intentional cases:
- `interpolate.cpp` 79% — residual is the `CubicSpline1D` Rule-of-3 deep-copy
  loops + spline-internal numeric branches (effectively at target).
- `trajectory_solver.cpp` 51% — the full boost::odeint solve + CSV dump are
  integration-level and covered by the Python E2E suite; only the constructor
  is unit-smoke-tested here.

Tests are organized by layer: utility/math, environment, rocket parameters,
rocket/engine, dynamics/solver, and Tier C (JSON control + factories, with
bad-input death tests). Tier B/C tests that need a built Rocket/Engine use the
shared builder in `test_fixtures.hpp`. `test_noniterative_iip.cpp.disabled` is
parked (its module is dead code — see below).

The production build is untouched by default. Tests live behind the
`BUILD_TESTING` CMake option (OFF by default), and the simulation sources are
compiled once into a `forrocket_core` static library shared by the `ForRocket`
executable, the tests, and coverage runs.

## One-time setup (GoogleTest)

GoogleTest is vendored the same way as Eigen/Boost/json, so builds stay offline:

```bash
cd lib && ./setup_googletest.sh
```

If you skip this, the configure step falls back to downloading GoogleTest via
CMake `FetchContent` (needs network). Pinned to `release-1.12.1`, the last line
that supports C++11 (the project standard).

## Build & run

```bash
cmake -S . -B build_test -DBUILD_TESTING=ON
cmake --build build_test --target forrocket_unittests -j
( cd build_test && ctest --output-on-failure )        # or:
./build_test/tests/unit/forrocket_unittests           # run the binary directly
```

> Run `ctest` from inside the build directory. `ctest --test-dir <dir>` needs
> CMake ≥ 3.20; this project targets 3.13+ (the dev box here has 3.16).

## Coverage

Needs `gcovr` (`pip install gcovr`). The helper script configures with
coverage, runs the tests, and writes an HTML + Cobertura-XML report:

```bash
tests/unit/run_coverage.sh
# HTML  -> build_coverage/coverage_report/index.html
# XML   -> build_coverage/coverage.xml   (Cobertura, for CI dashboards)
```

Manually, the only extra ingredient is `-DENABLE_COVERAGE=ON`, which adds
`-O0 -g --coverage` to `forrocket_core`.

## Adding a test

1. Create `test_<module>.cpp` here (`#include <gtest/gtest.h>` + the module header).
2. Add the filename to `forrocket_unittests` in [`CMakeLists.txt`](CMakeLists.txt).
3. Prefer `EXPECT_NEAR` for floating-point checks; assert against analytic
   reference values or invariants (round-trips, orthonormality) rather than
   recomputing the implementation.
4. For a module that needs a built Rocket/Engine, use `MakeTestRocket()` etc.
   from `test_fixtures.hpp` (read-only — don't edit it from a test).

> Coverage metric note: we track C1 via **branch** coverage
> (`gcovr --txt-metric branch --exclude-throw-branches`), NOT `gcovr --decisions`
> — the latter's experimental analyzer mis-scores consecutive single-line
> `if (cond) stmt;` blocks (it reported `flight_dynamics.cpp` 63% when true
> branch coverage was 100%).

Latent issues surfaced by these tests (not yet fixed; see project memory):
`noniterative_iip.cpp` is dead code with an uninitialized-read bug;
`interpolate.cpp:66` builds the cubic spline from unsorted data;
`RocketStage` ctor leaves `fdr.p_rocket` unwired.

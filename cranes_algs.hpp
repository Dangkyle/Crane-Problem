///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the TODO sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cassert>
#include <math.h>

#include "cranes_types.hpp"

namespace cranes {

// Solve the crane unloading problem for the given grid, using an exhaustive
// optimization algorithm.
//
// This algorithm is expected to run in exponential time, so the grid's
// width+height must be small enough to fit in a 64-bit int; this is enforced
// with an assertion.
//
// The grid must be non-empty.
path crane_unloading_exhaustive(const grid& setting) {

  // grid must be non-empty.
  assert(setting.rows() > 0);
  assert(setting.columns() > 0);

  // Compute maximum path length, and check that it is legal.
  const size_t max_steps = setting.rows() + setting.columns() - 2;
  assert(max_steps < 64);

  // TODO: implement the exhaustive search algorithm, then delete this
  // comment.
  path best(setting);
  for(size_t len = 0; len <= max_steps; len++){
            // for bits from 0 to 2^len - 1 inclusive
            for(size_t bits = 0; bits <= pow(2,len)-1; bits++){
                // candidate = [start]
                path candidate(setting);
                // for k from 0 to len-1 inclusive
                for(size_t k = 0; k < len; ++k){
                    int bit;
                    bit = (bits >> k) & 1;
                    // if bit is 1, then we will move →
                    if(bit == 1){
                        // make sure the candidate stays inside the grid and never crosses an X cell
                        if(candidate.is_step_valid(STEP_DIRECTION_EAST)){
                            // adding a right move to candidate
                            candidate.add_step(STEP_DIRECTION_EAST);
                        }
                     // else bit is 0, then we will move ↓
                    }else {
                        // make sure the candidate stays inside the grid and never crosses an X cell
                        if(candidate.is_step_valid(STEP_DIRECTION_SOUTH)){
                            // adding a down move to candidate
                            candidate.add_step(STEP_DIRECTION_SOUTH);
                        }
                    }
                }
                // if best is None or candidate harvests more gold than best
                if(best.last_step() == STEP_DIRECTION_START || candidate.total_cranes() > best.total_cranes()){
                    best = candidate;
                }
            }
        }
        return best;
    }

// Solve the crane unloading problem for the given grid, using a dynamic
// programming algorithm.
//
// The grid must be non-empty.
//path crane_unloading_dyn_prog(const grid& setting) {
path crane_unloading_dyn_prog(const grid& setting) {
  // grid must be non-empty.
  assert(setting.rows() > 0);
  assert(setting.columns() > 0);

  path best(setting);

  using cell_type = std::optional<path>;
  std::vector<std::vector<cell_type>> A(setting.rows(),
                                         std::vector<cell_type>(setting.columns()));

  A[0][0] = path(setting);
  assert(A[0][0].has_value());

  for (coordinate i = 0; i < setting.rows(); ++i) {
    for (coordinate j = 0; j < setting.columns(); ++j) {

      if (setting.get(i, j) == CELL_BUILDING) {
        A[i][j].reset();
        continue;
      }

      cell_type from_above = std::nullopt;
      cell_type from_left = std::nullopt;

      if (i > 0 && A[i - 1][j].has_value()) {
        // from_above = A[i-1][j] + [↓]
        from_above = A[i - 1][j];
        if (from_above->is_step_valid(STEP_DIRECTION_SOUTH)) {
          from_above->add_step(STEP_DIRECTION_SOUTH);
        }
      }
      if (j > 0 && A[i][j - 1].has_value()) {
        // from_left = A[i][j-1] + [→]
        from_left = A[i][j - 1];
        if (from_left->is_step_valid(STEP_DIRECTION_EAST)) {
          from_left->add_step(STEP_DIRECTION_EAST);
        }
      }
      if (from_above.has_value() && from_left.has_value()) {
        if (from_above->total_cranes() > from_left->total_cranes()) {
          A[i][j] = from_above;
        } else {
          A[i][j] = from_left;
        }
      } else if (from_above.has_value()) {
        A[i][j] = from_above;
      } else if (from_left.has_value()) {
        A[i][j] = from_left;
      }
    }
  }

  // post-processing to find maximum-cranes path
  if (A.back().back().has_value()) {
    best = *(A.back().back());
  }

  return best;
}

}

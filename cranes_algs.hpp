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
            for(size_t bits = 0; bits <= pow(2,len)-1; bits++){
                path candidate(setting);
                for(size_t k = 0; k < len; ++k){
                    int bit;
                    bit = (bits >> k) & 1;
                    if(bit == 1){
                        if(candidate.is_step_valid(STEP_DIRECTION_EAST)){
                            candidate.add_step(STEP_DIRECTION_EAST);
                        }
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

  using cell_type = std::optional<path>;

  std::vector<std::vector<cell_type>> A(setting.rows(),
                                        std::vector<cell_type>(setting.columns()));

  A[0][0] = path(setting);
  assert(A[0][0].has_value());

  //rubric.criterion("dynamic programming - random instances", 1, [&]() std::cout << std::endl;
  // auto small_output = crane_unloading_dyn_prog(small_random);
  //       TEST_EQUAL("small", 4, small_output.total_cranes());


  for (coordinate r = 0; r < setting.rows(); ++r) {
    for (coordinate c = 0; c < setting.columns(); ++c) {

      if (setting.get(r, c) == CELL_BUILDING) {
        A[r][c].reset();
        continue;
      }

      //begin if (setting.get(r, c) != CELL_BUILDING case
      //if (setting.get(r, c) != CELL_BUILDING) {

        // from_above = from_left = None
        std::optional<path> from_above;
        std::optional<path> from_left;

        // if r greater than 0 and ! None
        if (r > 0 && A[r-1][c].has_value()) {

           from_above = A[r-1][c].value();

           if(from_above->is_step_valid(STEP_DIRECTION_SOUTH) ) {
              from_above->add_step(STEP_DIRECTION_SOUTH);
           }
        }
        // if c greater than 0 and ! None
        if (c > 0 && A[r][c-1].has_value()) {

           from_left = A[r][c-1].value();

           if(from_left->is_step_valid(STEP_DIRECTION_EAST) ) {
              from_left->add_step(STEP_DIRECTION_EAST);
           }

        }
        // if from_above and from_left ! None
        if(from_above.has_value() && from_left.has_value()) {
          //if above > left
          if(from_above->total_cranes() > from_left->total_cranes()) {
            A[r][c] = from_above;
          }
          else
          {
            A[r][c] = from_left;
          }
        }
        // if from_above is non-None
        else if(from_above.has_value()) {
            A[r][c] = from_above;
        }
        // if from_left is non-None
        else if (from_left.has_value()){
           A[r][c] = from_left;
        }
      //end if (setting.get(r, c) != CELL_BUILDING case
    }
  }


  cell_type* best = &(A[0][0]);
  assert(best->has_value());
  for (coordinate r = 0; r < setting.rows(); ++r) {
    for (coordinate c = 0; c < setting.columns(); ++c) {
      if (A[r][c].has_value() && A[r][c]->total_cranes() > (*best)->total_cranes()) {
        best = &(A[r][c]);
      }
    }
  }

  assert(best->has_value());
 //   std::cout << "total cranes" << (**best).total_cranes() << std::endl;

  return **best;
}
}

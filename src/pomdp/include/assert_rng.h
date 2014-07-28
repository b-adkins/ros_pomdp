/*
 * @file
 *
 * Macro to standardize and simplify making range checks.
 *
 * @date Jul 12, 2013
 * @author Bea Adkins
 */

#ifndef ASSERT_RNG_H
#define ASSERT_RNG_H

#include <ros/assert.h>

/**
 * Assert designed to check index bounds.
 *
 * Passes if lo <= i < hi. Fails otherwise.
 *
 * @param i Index variable. (Integer.)
 * @param lo Lower bound.
 * @param hi Upper bound.
 * @param action Action to perform on failure.
 */
#define ROS_ASSERT_INDEX_CMD(i, lo, hi, action) \
  ROS_ASSERT_CMD((i >= lo && i < hi), \
                 { \
                   ROS_ERROR(#i": %i outside valid range [%i, %i)", i, lo, hi); \
                   action; \
                 } \
  );

/**
 * Assert designed to check value bounds.
 *
 * Passes if lo <= x <= hi. Fails otherwise.
 *
 * @param x Variable. (Floating point.)
 * @param lo Lower bound.
 * @param hi Upper bound.
 * @param action Action to perform on failure.
 */
#define ROS_ASSERT_RNG_CMD(x, lo, hi, action) \
  ROS_ASSERT_CMD((x >= lo && x <= hi), \
                 { \
                   ROS_ERROR(#x": %f outside valid range (%f, %f)", x, lo, hi); \
                   action; \
                 } \
  );


#endif /* ASSERT_RNG_H */

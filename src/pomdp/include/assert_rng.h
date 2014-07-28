/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Boone "Bea" Adkins
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
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

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
/**
 * @file
 *
 * Macros to check mathematical conditions.
 *
 * @date Jul 19, 2013
 * @author Bea Adkins
 */

#ifndef ASSERT_MATH_H
#define ASSERT_MATH_H

#include <algorithm>
#include <ros/assert.h>

/**
 * Checks that contents of container add up to within an epsilon of an expected sum.
 *
 * Prints a message on failure.
 *
 * @param container Container with values to be summed.
 * @param sum Expected sum.
 * @param ep Epsilon for comparing with sum.
 * @param cmd Command to run on failure.
 *
 * @todo Not yet tested for integer types. Templates may help?
 */
#define ROS_ASSERT_SUM_CMD(container, sum, ep, cmd) \
{\
  double actual_sum = std::accumulate(container.begin(), container.end(), 0.0); \
  ROS_ASSERT_CMD((fabs(actual_sum - sum) < ep), \
                 ROS_ERROR_STREAM("Expected sum("#container") = " << sum << ", actual value: " << actual_sum); \
                 cmd;); \
}

#define ROS_ASSERT_VALID_PMF_CMD(container, ep, cmd) \
  ROS_ASSERT_SUM_CMD(container, 1.0, ep, cmd)

#endif /* ASSERT_MATH_H */

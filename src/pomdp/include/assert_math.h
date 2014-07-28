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

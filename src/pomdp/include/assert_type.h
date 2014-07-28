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
 * Assertions for run-time (preferably during loading, before main execution)
 * type checking.
 *
 * @date Jul 24, 2013
 * @author Bea Adkins
 */

#ifndef ASSERT_TYPE_H
#define ASSERT_TYPE_H

#include <typeinfo>

#include <ros/assert.h>
#include <ros/console.h>
#include <boost/units/detail/utility.hpp>

/**
 * Asserts that "var" is of type "type".
 *
 * @param var Object to typecheck.
 * @param type Type (ClassName).
 * @param cmd Command to execute on failure. (In addition to error message.)
 */
#define ROS_ASSERT_TYPE_CMD(var, type, cmd)                             \
  ROS_ASSERT_CMD((typeid(var) == typeid(type)),                         \
                 ROS_ERROR("'"#var"' is a '%s', not a '%s'.",           \
                           boost::units::detail::demangle(typeid(var).name()).c_str(), \
                           boost::units::detail::demangle(typeid(type).name()).c_str()); \
                 cmd;                                                   \
                );

/**
 * Asserts that "var" is of type "type".
 *
 * @param var Variable to typecheck.
 * @param type Type (ClassName).
 */
#define ROS_ASSERT_TYPE(var, type) ROS_ASSERT_TYPE_CMD(var, type, ROS_ISSUE_BREAK())

/**
 * Asserts that pointer "var" is of type "type" or a child of type "type".
 *
 * @param var Pointer / std::shared_ptr / boost::shared_ptr to object to typecheck.
 * @param type Type (ClassName), without pointer.
 * @param cmd Command to execute on failure. (In addition to error message.)
 */
#define ROS_ASSERT_CHILD_CMD(var, type, cmd)\
    ROS_ASSERT_CMD((var), ROS_ERROR("'"#var"' is NULL/empty!"); cmd;); /* Fails for NULL or empty shared_ptr's*/\
    try{ const type& test = dynamic_cast<const type&>(*var); /* Verified dereferenceable. */} \
    catch(std::bad_cast& e) /* Catch std::bad_cast for failure condition. */ \
    {\
      ROS_ERROR("'"#var"' is a '%s', not a child of '%s'.",           \
                boost::units::detail::demangle(typeid(var).name()).c_str(), \
                boost::units::detail::demangle(typeid(type).name()).c_str()); \
      cmd;                                                   \
    }

/**
 * Asserts that pointer "var" is of type "type" or a child of type "type".
 *
 * @param var Pointer to object to typecheck.
 * @param type Type (ClassName), without pointer.
 */
#define ROS_ASSERT_CHILD(var, type) ROS_ASSERT_TYPE_CMD(var, type, ROS_ISSUE_BREAK())

#endif /* ASSERT_TYPE_H */

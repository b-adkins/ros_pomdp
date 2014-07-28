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
 *
 *
 * @date Aug 13, 2013
 * @author Bea Adkins
 */

#ifndef POMDP_IO_H
#define POMDP_IO_H

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

namespace pomdp
{

/**
 * Detect lines that are all whitespace.
 *
 * @return True if line is blank.
 */
inline bool isBlank(const std::string& s)
{
  return boost::algorithm::all(s, boost::algorithm::is_space());
}

/**
 * Strips inline comments from the end of a line.
 *
 * E.g.
 * Will be preserved. # Will be stripped.
 * Will be preserved as well. // Will be stripped.
 *
 * @param s String to be stripped.
 * @param marker Character or string that marks an inline comment, e.g. '#', '%', "//".
 * @return New string with comment (if one existed) stripped.
 */
inline std::string stripComment(const std::string& s, const std::string& marker = "#")
{
  std::string tmp(s);
  boost::regex inline_comment_regex(marker + ".*$");

  return boost::regex_replace(tmp, inline_comment_regex, "", boost::match_default | boost::format_all);
}

int getNumberFromTagColonLine(const std::string& path, const std::string& tag);
std::string getStringFromTagColonLine(const std::string& path, const std::string& tag);

} /* namespace pomdp */
#endif /* POMDP_IO_H */

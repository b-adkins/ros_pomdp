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
 * @date Aug 16, 2013
 * @author Bea Adkins
 */

#include "pomdp_io.h"

namespace pomdp
{

/**
 * Searches for a line of the format "tag: [int]"), e.g. "actions: 14" and reads the int.
 *
 * @param path Path to the file.
 * @param tag Tag (minus the colon), e.g. "tag".
 * @return Number to the right of the tag, e.g. 14 or -1 on failure.
 */
int getNumberFromTagColonLine(const std::string& path, const std::string& tag)
{
  int ret;

  // Open file
  std::ifstream num_tag_file;
  num_tag_file.open(path.c_str());
  if(!num_tag_file.good())
    return -1;

  // Read tag.
  std::string line;
  boost::regex tagged_line("^\\s*" + tag + ":\\s+(\\d+)"); // Int in parens, extracted as subexpression match later.
  boost::smatch tagged_line_match; // Submatch which will only hold the number.
  while(!num_tag_file.eof())
  {
    std::getline(num_tag_file, line);
    if(num_tag_file.fail())
      return -1;

    if(boost::regex_match(line, tagged_line_match, tagged_line)) // Tag found.
    {
      std::string number_str = tagged_line_match[1].str(); // Match to first subexpression.
      std::stringstream number_ss(number_str);
      if(number_ss >> ret) // Number read successfully. We're done here.
        return ret;
      else                 // Failed to read number.
        return -1;
    }
  }

  return -1;             // Tag not found. Fail.
}

/**
 * Searches for a line of the format "tag: [string]"), e.g. "actions: left right forward" and reads the string.
 *
 * @param path Path to the file.
 * @param tag Tag (minus the colon), e.g. "tag".
 * @return String to the right of the tag, e.g. "left right forward" or empty string ("") on failure.
 */
std::string getStringFromTagColonLine(const std::string& path, const std::string& tag)
{
  int ret;

  // Open file
  std::ifstream num_tag_file;
  num_tag_file.open(path.c_str());
  if(!num_tag_file.good())
    return "";

  // Read tag.
  std::string line;
  boost::regex tagged_line("^\\s*" + tag + ":\\s+(.+)\\s*(#|$)"); // String in parens, extracted as subexpression match later.
  boost::smatch tagged_line_match; // Submatch which will only hold the number.
  while(!num_tag_file.eof())
  {
    std::getline(num_tag_file, line);
    if(num_tag_file.fail())
      return "";

    if(boost::regex_match(line, tagged_line_match, tagged_line)) // Tag found.
    {
      std::string str = tagged_line_match[1].str(); // Match to first subexpression.
      return str; // We're done here.
    }
  }

  return "";             // Tag not found. Fail.
}

} /* namespace pomdp */

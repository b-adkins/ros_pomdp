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
#include <boost/make_shared.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

#include "pretty.h"

namespace pomdp
{

Pretty::Pretty()
{

}

/**
 * Copy constructor.
 */
Pretty::Pretty(const Pretty& p)
{
  this->pretty_map_ = p.pretty_map_;
}

/**
 * Takes numerical mappings from vector position (starting at '0').
 *
 * @param names Vector of names.
 */
Pretty::Pretty(const std::vector<std::string>& names)
{
  for(int i = 0; i < names.size(); i++)
  {
    addMapping(names[i], i);
  }
}

/**
 * Uses numerical mappings provided. Ignores duplicates.
 *
 * @param names Vector of names.
 * @param numbers Vector of corresponding numbers.
 */
Pretty::Pretty(const std::vector<std::string>& names, const std::vector<int>& numbers)
{
  int i_max = names.size() < numbers.size() ? names.size() : numbers.size();
  for(int i = 0; i < i_max; i++)
  {
    addMapping(names[i], numbers[i]);
  }
}

/**
 * Loads a pretty print mapping from a file.
 *
 * See Pretty::loadNames() for details.
 *
 * @param path
 * @param enum_name
 */
Pretty::Pretty(const std::string& path, const std::string& enum_name)
{
  this->loadNames(path, enum_name);
}

Pretty::~Pretty()
{
}

/**
 * Loads a pretty print mapping from a file.
 *
 * Maps to names to sequential integers starting at '0'.
 * 
 * Can call with an existing Pretty object (adds to existing mapping).
 * @todo Make this actually work by starting int mapping from max existing value.
 *
 * Reads enum from a line of form:
 * @code quark_flavor: up down strange charm bottom top @endcode
 *
 * Into a pretty printer with the following mapping:
 * @code {up:0, down:1, strange:2, charm:3, bottom:4, top:5} @endcode
 *
 * Fails for a line of form:
 * @code quark_flavor: 6 @endcode
 * (Contains only
 *
 * @param path File path.
 * @param enum_name The name of this mapping (which heads the line), without the ':'. E.g. 'quark_flavor' above.
 * @return True on success.
 */
bool Pretty::loadNames(const std::string& path, const std::string& enum_name)
{
// Open file.
  std::ifstream enum_file;
  enum_file.open(path.c_str());
  if(!enum_file.good())
    return false;

  // Look for enum line.
  std::string line;
  std::string enum_tag = enum_name + ':'; // @todo Test enum_name for matching alphanumeric and '_' restriction.
  std::size_t enum_tag_pos; //4noobs Position in which we found enum_tag
  bool tag_found = false; //4noobs Did we find enum_tag in the file?
  while(!enum_file.eof()) //4noobs while we haven't reached end of file.
  {
    std::getline(enum_file, line);
    if(enum_file.fail())
      return false;

    if(line.compare(0, enum_tag.size(), enum_tag) == 0) // @todo Regular expression match? Or don't bother (too simple a format).
    {
      tag_found = true;
      break;    //4noobs We've found the desired line, so stop the loop and parse it.
    }
  }
  // Fail if no match.
  if(!tag_found)
    return false;


  // Parse enum line.
  std::string name; //4noobs Temporary vector to hold a single name.
  std::vector<std::string> names; //4noobs Vector of names, to be passed to Pretty constructor.
  std::stringstream line_ss(line); //4noobs Create string stream to manipulate the line.
  line_ss >> name; // Throw away the enum_name (no longer needed)
  while(line_ss >> name) //4noobs Read all the names (trims whitespace automatically)
  {
    names.push_back(name); //4noobs Add them to the names vector.
  }
  // Fail if no names were read.
  if(names.size() == 0)
    return false;
  // Fail if line only read a single integer.
  if(names.size() == 1)
  {
    // Kludgy "is this an int?" test.
    std::stringstream first_name_ss(names[0]);
    int test_int;
    if(first_name_ss >> test_int) //4noobs It is an int.
    {
      return false;
    }
  }

  // Add names.
  bool success = true;
  for(int i = 0; i < names.size(); i++)
  {
    if(!addMapping(names[i], i)) //4noobs If adding a mapping failed, the whole thing fails.
      success = false;
  }

  return success;
}

// Static so that it can be used without a Pretty object.
/**
 * Turns input string into matching number, either directly (if it's numeric) or by looking up the mapping.
 *
 * @param number_or_name Input string (e.g. "14" or "go_forward")
 * @param mapping Pretty printer. If empty pointer passed, only reads s as an integer, doesn't attempt name lookup.
 * @return Number or -1 for failure.
 */
// static
int Pretty::readNameOrNum(const std::string& number_or_name, shared_ptr<const Pretty> mapping)
{
  std::stringstream number_or_name_ss(number_or_name);
  int number;

  // Try reading it as an integer.
  number_or_name_ss >> number;
  if(!number_or_name_ss.fail())
    return number;  // Successfully read int.
  else /* line_ss.fail */
  {
    // No mapping to use strings with.
    if(!mapping)
      return -1;

    // Try reading it as a name corresponding to a number.
    return mapping->toEnum(number_or_name_ss.str());
  }
}

/**
 * Converts the pretty print map itself into a string.
 */
std::ostream& operator<<(std::ostream& os, const Pretty& pretty)
{
  os << "{ ";
  for(std::map<int, std::string>::const_iterator itr = pretty.pretty_map_.begin();
      itr != pretty.pretty_map_.end();
      itr++)
  {
    os << boost::lexical_cast<std::string>(itr->first) << ":" << itr->second << " ";
  }
  os << "}";
  return os;
}

/**
 * Converts a number into a name.
 *
 * @param number Enumeration value.
 * @return Name contained in map or printed number if not found.
 */
std::string Pretty::toString(int number) const
{
  if(pretty_map_.find(number) != pretty_map_.end())
  {
    std::string ret = pretty_map_.at(number);
    return ret;
  }
  else
  {
    return boost::lexical_cast<std::string>(number);
  }
}

/**
 * Converts a name into a number.
 *
 * @param name Name of value.
 * @return Value contained in map or -1 if not found.
 *
 * @todo If this - O(n) - is too slow, replace with boost::bimap - O(1) - but 2x bigger object. Otherwise, don't bother.
 */
int Pretty::toEnum(const std::string& name) const
{
  for(std::map<int, std::string>::const_iterator itr = pretty_map_.begin();
      itr != pretty_map_.end();
      itr++)
  {
    if(itr->second == name)
      return itr->first;
  }

  return -1;
}

/**
 * Adds a mapping. Fails on duplicates.
 *
 * @param name Nominal value.
 * @param number Numeric value.
 * @return True if mapping was added.
 */
bool Pretty::addMapping(const std::string& name, int number)
{
  if(pretty_map_.find(number) != pretty_map_.end())
  {
    return false;
  }
  else
  {
    pretty_map_[number] = name;
    return true;
  }

}

std::vector<std::string> Pretty::getNames() const
{
  std::vector<std::string> ret;

  int count = 0;
  for(std::map<int, std::string>::const_iterator itr = pretty_map_.begin();
      itr != pretty_map_.end();
      itr++, count++)
  {
    int key = itr->first;
    const std::string& name = itr->second;

    if(key != count)
      return std::vector<std::string>();
    else
      ret.push_back(name);
  }

  return std::vector<std::string>(ret);
}

// friend
bool operator==(const Pretty& a, const Pretty& b)
{
  return a.pretty_map_ == b.pretty_map_;
}

// static
const shared_ptr<Pretty> Pretty::empty = boost::make_shared<Pretty>();

} /* namespace pomdp */

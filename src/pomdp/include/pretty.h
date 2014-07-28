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
 * @class pomdp::Pretty
 *
 * Pretty printer for any enumeration-like quantities.
 *
 * Maps a set of integers to a corresponding set of strings. Is robust: if a mapping is not available (or none are),
 * prints the integer.
 *
 * One object per enumeration.
 *
 * @date Jul 20, 2013
 * @author Bea Adkins
 */

#ifndef PRETTY_PRINT_POMDP_H
#define PRETTY_PRINT_POMDP_H

#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using boost::shared_ptr; // Because it's long and ugly enough v.s. a '*'.

namespace pomdp
{

class Pretty
{
public:
  Pretty();
  Pretty(const Pretty& p);
  Pretty(const std::vector<std::string>& names);
  Pretty(const std::vector<std::string>& names, const std::vector<int>& numbers);
  Pretty(const std::string& path, const std::string& enum_name);
  virtual ~Pretty();

  bool loadNames(const std::string& path, const std::string& enum_name);
  static int readNameOrNum(const std::string& name, shared_ptr<const Pretty> p = shared_ptr<const Pretty>());

  std::string toString() const;
  friend std::ostream& operator<<(std::ostream& os, const Pretty& pretty);
  std::string toString(int x) const;
  int toEnum(const std::string& name) const;

  template<typename T> static std::string arrayToString(int n, const T* a, const std::string& sep = ", ");
  template<typename T> static std::string vectorToString(const std::vector<T>& v, const std::string& sep = ", ");

  friend bool operator==(const Pretty& a, const Pretty& b);
  bool addMapping(const std::string& name, int number);

  /**
   * @return Names stored within, sans mapping. Empty if the mapping is not consecutive.
   */
  std::vector<std::string> getNames() const;

  /**
   * @return Number of number:name mappings inside this pretty printer.
   */
  int size() const
  {
    return pretty_map_.size();
  }

  /**
   * Empty pretty printer containing no mappings. Only reads/writes numbers.
   */
  const static shared_ptr<Pretty> empty;
private:
  std::map<int,std::string> pretty_map_;
};

/**
 * Pretty prints an array to a string.
 *
 * @tparam T Type contained within array.
 * @param n Number of elements in array.
 * @param a C array of type T.
 * @param sep Seperator printed between vector elements.
 * @return String form of array (or "NULL" if null).
 */
template<typename T>
std::string Pretty::arrayToString(int n, const T* a, const std::string& sep)
{
  if(a == NULL)
    return "NULL";

  typename std::vector<T> array_v;
  array_v.assign(a, a + n);
  return vectorToString(array_v, sep);
}

/**
 * Pretty prints a vector to a string.
 *
 * @tparam T Type contained within vector.
 * @param v STL vector of type T.
 * @param sep Seperator printed between vector elements.
 * @return String form of vector.
 */
template<typename T>
std::string Pretty::vectorToString(const std::vector<T>& v, const std::string& sep)
{
  std::ostringstream ret;
  ret << std::setprecision(4);
  ret << std::fixed;

  ret << "[";
  for(int i = 0; i < v.size() - 1; i++)
  {
    ret << v[i] << sep;
  }
  ret << v[v.size() - 1]; // Prints last element without seperator.
  ret << "]";

  return ret.str();
}

} /* namespace pomdp */
#endif /* PRETTY_PRINT_POMDP_H */

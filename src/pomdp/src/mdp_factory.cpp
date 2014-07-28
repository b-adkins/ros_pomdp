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
 * @date Aug 12, 2013
 * @author Bea Adkins
 */

#include <boost/filesystem/path.hpp>

#include "mdp_factory.h"
#include "mdp.h"
#include "pomdp.h"
#include "empty_mdp.h"


namespace pomdp
{

// static
MDP* MDPFactory::loadFromFile(const std::string& path)
{
  MDP* mdp_new;

  // Parse file path.
  boost::filesystem::path boost_path(path);
  std::string ext = boost_path.extension().string();
  std::string stem = boost_path.stem().string();

  if(ext == "")
  {
    ROS_ERROR("Error: extension needed for (PO)MDP file '%s'", stem.c_str());
    return false;
  }

  if(ext == ".mdp" || ext == ".MDP")
    mdp_new = new MDP();
  else if(ext == ".empty_mdp")
    mdp_new = new EmptyMDP();
  else if(ext == ".pomdp" || ext == ".POMDP")
    mdp_new = new POMDP();
  // Developers: insert new (PO)MDP classes here.
  else
  {
    ROS_ERROR("Unrecognized file extension '%s'.", ext.c_str());
    return NULL;
  }

  // Load the MDP from the config file.
  if(!mdp_new->loadFromFile(path))
    return NULL;
  mdp_new->setName(stem);

  ROS_DEBUG("Loaded (PO)MDP '%s'.", mdp_new->getName().c_str());

  return mdp_new;
}

} /* namespace pomdp */

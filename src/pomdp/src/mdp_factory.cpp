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

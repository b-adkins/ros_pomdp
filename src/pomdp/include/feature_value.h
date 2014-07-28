/**
 * @file
 *
 * Abstract superclass for states/actions/observations.
 *
 * @date Sep 7, 2013
 * @author Bea Adkins
 */

#ifndef FEATURE_VALUE_H
#define FEATURE_VALUE_H

#include <string>

namespace pomdp
{

class FeatureValue
{
public:
  FeatureValue(){};
  virtual ~FeatureValue(){};
};

} /* namespace pomdp */
#endif /* FEATURE_VALUE_H */

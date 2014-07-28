/**
 * @file
 *
 *
 *
 * @date Aug 12, 2013
 * @author B. Adkins
 */

#include "pretty.h"
#include "simple_feature_value.h"

namespace pomdp
{

SimpleFeatureValue::SimpleFeatureValue(unsigned int state)
{
  state_ = state;
}

SimpleFeatureValue::SimpleFeatureValue(SimpleFeatureValue const& s)
{
  *this = s;
}

SimpleFeatureValue& SimpleFeatureValue::operator=(SimpleFeatureValue const& s)
{
  if(this != &s)
    state_ = s.asInt();

 return *this;
}

SimpleFeatureValue& SimpleFeatureValue::operator=(unsigned int const& s)
{
  return operator=(SimpleFeatureValue(s));
}

} /* namespace pomdp */

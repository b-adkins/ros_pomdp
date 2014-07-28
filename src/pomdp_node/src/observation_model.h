/**
 * @class pomdp::ObservationModel
 *
 *
 *
 * @date Jul 19, 2013
 * @author Bea Adkins
 */

#ifndef OBSERVATION_MODEL_H
#define OBSERVATION_MODEL_H

#include <string>

namespace pomdp
{

class ObservationModel
{
public:
  ObservationModel();
  virtual ~ObservationModel();

  bool isInit() const{ return is_init_; };
  std::string getName() const{ return name_; };
  void setName(std::string name){ name_ = name; };
protected:
  bool is_init_;
  std::string name_;
};

} /* namespace pomdp */
#endif /* OBSERVATION_MODEL_H */

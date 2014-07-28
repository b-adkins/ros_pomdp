/**
 * @class pomdp::MDPFactory
 *
 * Class for creating MDPs.
 *
 * @date Aug 12, 2013
 * @author Bea Adkins
 */

#ifndef MDP_FACTORY_H
#define MDP_FACTORY_H

namespace pomdp
{
class MDP; // No need to tangle dependencies.

class MDPFactory
{
public:
  static MDP* loadFromFile(const std::string& path);
};

} /* namespace pomdp */
#endif /* MDP_FACTORY_H */

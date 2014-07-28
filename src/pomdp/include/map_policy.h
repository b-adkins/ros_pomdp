/**
 * @class pomdp::MapPolicy
 *
 * MDP policy using direct map from state to action.
 *
 * Uses a very simple file format. State-actions pairs are listed one per line,
 * where the state number is the line number (starting with 0).
 *
 * @code
 * # Inline comments using hashes.
 * #
 * # E.g.
 * east  # (0, 0)
 * south # (0, 1)
 * # ...etc
 * @endcode
 *
 * @date Jul 12, 2013
 * @author Bea Adkins
 */

#ifndef MAP_POLICY_H
#define MAP_POLICY_H

#include "assert_rng.h"
#include "mdp_policy.h"

namespace pomdp
{

class MapPolicy : public MDPPolicy
{
public:
  MapPolicy();
  MapPolicy(const char* path);
  virtual bool loadFromFile(const char* path);
  using Policy::loadFromFile;
  virtual ~MapPolicy();

  virtual int applyPolicy(const FeatureValue& index) const;
};

} /* namespace pomdp */

#endif /* MAP_POLICY_H */

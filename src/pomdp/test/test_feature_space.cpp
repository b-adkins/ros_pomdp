/**
 * @file
 *
 * Tests for pomdp::FeatureVector and its associated classes.
 *
 * Uses c++0x initializers for vector values.
 *
 * @date Aug 20, 2013
 * @author Bea Adkins
 */

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>
#include <sstream>

#include "belief_feature_value.h"
#include "simple_feature_space.h"
#include "simple_feature_value.h"
#include "pretty.h"
#include "vector_feature_space.h"

using namespace pomdp;

/**
 * Reimplementation of YAML::Node.Load() from 5.x+ yaml-cpp, to be replaced with that when ROS switches over.
 *
 * @param str Valid YAML.
 * @return First root node of given YAML.
 */
std::auto_ptr<YAML::Node> YAML_Node_Load(std::string str)
{
  YAML::Node ret;

  std::istringstream str_ss(str);
  YAML::Parser parser(str_ss);
  parser.GetNextDocument(ret);
  return ret.Clone();
}

class TestSimpleFeatureSpace : public ::testing::Test
{
protected:
  SimpleFeatureSpace* color;
  SimpleFeatureSpace* number;

  virtual void SetUp()
  {
    std::auto_ptr<YAML::Node> palette = YAML_Node_Load("{name: 'color', values: ['red', 'blue', 'yellow']}");
    color = new SimpleFeatureSpace();
    ASSERT_TRUE(color->loadFromYAML(*palette));

    std::auto_ptr<YAML::Node> numeric_range = YAML_Node_Load("name: 'x'\nvalues: 9\n");
    number = new SimpleFeatureSpace();
    ASSERT_TRUE(number->loadFromYAML(*numeric_range));
  }

  virtual void TearDown()
  {
    delete color;
    delete number;
  }
};

TEST_F(TestSimpleFeatureSpace, testLoadTooShort)
{
  SimpleFeatureSpace fs;
  std::auto_ptr<YAML::Node> param = YAML_Node_Load("{name: 'ynt-sized', values: 1}");
  EXPECT_FALSE(fs.loadFromYAML(*param));
}

TEST_F(TestSimpleFeatureSpace, testLoadTooNegative)
{
  SimpleFeatureSpace fs;
  std::auto_ptr<YAML::Node> param = YAML_Node_Load("{name: 'zany', values: -499}");
  EXPECT_FALSE(fs.loadFromYAML(*param));
}

TEST_F(TestSimpleFeatureSpace, testNumValues)
{
  EXPECT_EQ(3, color->getNumValues());
  EXPECT_EQ(9, number->getNumValues());
}

TEST_F(TestSimpleFeatureSpace, testValueValid)
{
//  EXPECT_TRUE(color->isValid("red"));
//  EXPECT_TRUE(color->isValid("blue"));
//  EXPECT_TRUE(color->isValid("yellow"));
  EXPECT_TRUE(color->isValid(SimpleFeatureValue(0)));
  EXPECT_TRUE(color->isValid(SimpleFeatureValue(2)));

  EXPECT_TRUE(number->isValid(SimpleFeatureValue(0)));
  EXPECT_TRUE(number->isValid(SimpleFeatureValue(8)));
}

TEST_F(TestSimpleFeatureSpace, testValueInvalidOutOfRange)
{
//  EXPECT_FALSE(color->isValid("purple"));
//  EXPECT_FALSE(color->isValid("green"));
  EXPECT_FALSE(color->isValid(SimpleFeatureValue(3)));

  EXPECT_FALSE(number->isValid(SimpleFeatureValue(9)));
}

TEST_F(TestSimpleFeatureSpace, testValueInvalid)
{
//  EXPECT_FALSE(color->isValid(("red", 2)));
  EXPECT_FALSE(color->isValid(FeatureVectorValue({4, 3})));

//  EXPECT_FALSE(number->isValid("one"))
//  EXPECT_FALSE(number->isValid(("red", 2)));
  EXPECT_FALSE(number->isValid(FeatureVectorValue({4, 3})));
}

class TestVectorFeatureSpace : public ::testing::Test
{
protected:
  VectorFeatureSpace* xy;
  VectorFeatureSpace* xycolor;

  virtual void SetUp()
  {
    std::auto_ptr<YAML::Node> coordinates = YAML_Node_Load("- {name: x, values: 3}\n"
                                                           "- {name: y, values: 2}");
    xy = new VectorFeatureSpace();
    ASSERT_TRUE(xy->loadFromYAML(*coordinates));

    std::auto_ptr<YAML::Node> pixels = YAML_Node_Load("- {name:     x, values: 3}\n"
                                                      "- {name:     y, values: 2}\n"
                                                      "- {name: color, values: 4}");
    xycolor = new VectorFeatureSpace();
    ASSERT_TRUE(xycolor->loadFromYAML(*pixels));
  }

  virtual void TearDown()
  {
    delete xy;
    delete xycolor;
  }
};

//
// Test data, initializer lists for FeatureVectorValues (aka std::vectors).
//
// Macros so that the values show up in GTest messages.
//
#define zero FeatureVectorValue({}) // Degenerate case.
#define one FeatureVectorValue({0}) // Value zero is valid for any sized state, thus this tests sizing.

                                              /* x  y */
#define xy_min_range         FeatureVectorValue({0, 0})
#define xy_max_range         FeatureVectorValue({2, 1})
#define xy_outside_range_0   FeatureVectorValue({3, 1})
#define xy_outside_range_1   FeatureVectorValue({0, 4})
                                                   /* x  y color */
#define xycolor_min_range         FeatureVectorValue({0, 0, 0})
#define xycolor_max_range         FeatureVectorValue({2, 1, 3})
#define xycolor_outside_range_2   FeatureVectorValue({0, 0, 4})

TEST_F(TestVectorFeatureSpace, testValueValid)
{
  EXPECT_TRUE(xy->isValid(xy_min_range));
  EXPECT_TRUE(xy->isValid(xy_max_range));

  EXPECT_TRUE(xycolor->isValid(xycolor_min_range));
  EXPECT_TRUE(xycolor->isValid(xycolor_max_range));
}

TEST_F(TestVectorFeatureSpace, testValueInvalidSize)
{
  EXPECT_FALSE(xy->isValid(zero));
  EXPECT_FALSE(xy->isValid(one));
  EXPECT_FALSE(xy->isValid(xycolor_min_range));
  EXPECT_FALSE(xy->isValid(xycolor_max_range));
  EXPECT_FALSE(xy->isValid(xycolor_outside_range_2));
}

TEST_F(TestVectorFeatureSpace, testValueInvalidOutOfRange)
{
  EXPECT_FALSE(xy->isValid(xy_outside_range_0));
  EXPECT_FALSE(xy->isValid(xy_outside_range_1));
  EXPECT_FALSE(xy->isValid(xycolor_outside_range_2));
}

TEST_F(TestVectorFeatureSpace, testHash)
{
  EXPECT_EQ(0, xy->hash(xy_min_range).asInt());
  EXPECT_EQ(4, xy->hash(FeatureVectorValue({1, 1})).asInt());
  EXPECT_EQ(5, xy->hash(xy_max_range).asInt());

  EXPECT_EQ(0, xycolor->hash(xycolor_min_range).asInt());
  EXPECT_EQ(11, xycolor->hash(FeatureVectorValue({2, 1, 1})).asInt());
  EXPECT_EQ(15, xycolor->hash(FeatureVectorValue({0, 1, 2})).asInt());
  EXPECT_EQ(23, xycolor->hash(xycolor_max_range).asInt());
}

TEST_F(TestVectorFeatureSpace, testDehash)
{
  FeatureVectorValue expected({});

  EXPECT_EQ((expected = xy_min_range), xy->dehash(SimpleFeatureValue(0)));
  EXPECT_EQ((expected = FeatureVectorValue({1, 1})), xy->dehash(SimpleFeatureValue(4)));
  EXPECT_EQ((expected = xy_max_range), xy->dehash(SimpleFeatureValue(5)));

  EXPECT_EQ((expected = xycolor_min_range), xycolor->dehash(SimpleFeatureValue(0)));
  EXPECT_EQ((expected = FeatureVectorValue({2, 1, 1})), xycolor->dehash(SimpleFeatureValue(11)));
  EXPECT_EQ((expected = FeatureVectorValue({0, 1, 2})), xycolor->dehash(SimpleFeatureValue(15)));
  EXPECT_EQ((expected = xycolor_max_range), xycolor->dehash(SimpleFeatureValue(23)));
}

// Currently, VectorFeatureSpaces are immutable
//TEST_F(TestFeatureVector, testAddFeature)
//{
//  VectorFeatureSpace my_xy(*xy); // Make local copy.
//  Feature transparency = {"transparency",
//                          boost::make_shared<Pretty>(std::vector<std::string>({"opaque", "translucent",
//                                                       "transparent"}))};
//
//  // Test the find portion of adding. (Succeeds if doesn't exist, fails if does exist.)
//  EXPECT_TRUE(xy->addFeature(transparency));
//  EXPECT_FALSE(xy->addFeature(transparency)); // Duplicate forbidden.
//
//  // Tests that feature was properly added.
//  ASSERT_EQ(3, xy->size()); // No reason to continue if the size is wrong.
//  EXPECT_EQ(16, xy->hash(FeatureVectorValue({1, 1, 2}))); // Test that numeric state is still calculated properly.
//}

// Currently, VectorFeatureSpaces are immutable
// Doesn't actually use the text fixture, but no reason to give it its own test case.
//TEST_F(TestFeatureVector, testRemoveFeature)
//{
//  Feature tex = {"texture",
//                 boost::make_shared<Pretty>(std::vector<std::string>({"fuzzy", "prickly"}))};
//  Feature size = {"size",
//                  boost::make_shared<Pretty>(std::vector<std::string>({"tiny", "small", "medium", "large",
//                                               "extra_large", "huge", "prodigious", "ginormous"}))};
//  FeatureVector has_two({ tex, size });
//  Feature has_not; // Empty Feature.
//
//  // Should have been initialized properly.
//  ASSERT_EQ(2, has_two.size());
//  EXPECT_EQ(11, has_two.toStateNumber(FeatureVectorValue({1, 5})));
//
//  // Can't remove a feature that doesn't exist!
//  ASSERT_FALSE(has_two.removeFeature(has_not));
//  ASSERT_EQ(2, has_two.size());
//  EXPECT_EQ(11, has_two.toStateNumber(FeatureVectorValue({1, 5}))); // Should not have messed up calculations.
//
//  // Remove same object given.
//  ASSERT_TRUE(has_two.removeFeature(tex));
//  ASSERT_EQ(1, has_two.size());
//  EXPECT_EQ(6, has_two.toStateNumber(FeatureVectorValue({6})));
//
//  // Remove copy of object given.
//  Feature size_copy(size);
//  ASSERT_TRUE(has_two.removeFeature(size_copy));
//  ASSERT_EQ(0, has_two.size());
//}

// FeatureVectorFactory deprecated, replace with test for VectorFeatureSpace factory
//TEST(TestFeatureVectorIO, testLoadFromFile)
//{
//  FeatureVectorFactory loader;
//  FeatureVector fv = loader.loadFromFile("../../src/pomdp/test/feature_vector.yaml");
//
//  ASSERT_TRUE(fv.size() == 4); // Feature vector must have correct size for the rest of the tests to work.
//  EXPECT_TRUE(fv.validValue(FeatureVectorValue({1, 4, 8, 8}))); // Max bounds of each Feature.
//  EXPECT_FALSE(fv.validValue(FeatureVectorValue({2, 5, 9, 9}))); // Just outside max bounds of each Feature.
//}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

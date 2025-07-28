/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <RBDyn/parsers/yaml.h>

#include <gtest/gtest.h>

// Test utilties
#include "ParsersTestUtils.h"

TEST(YAMLParserTest, loadTest)
{
  auto cppRobot = createRobot();
  auto strRobot = rbd::parsers::from_yaml(XYZSarmYaml);

  EXPECT_EQ(cppRobot.mb.nrBodies(), strRobot.mb.nrBodies());
  EXPECT_EQ(cppRobot.mb.nrJoints(), strRobot.mb.nrJoints());
  EXPECT_EQ(cppRobot.mb.nrParams(), strRobot.mb.nrParams());
  EXPECT_EQ(cppRobot.mb.nrDof(), strRobot.mb.nrDof());

  ASSERT_TRUE(std::equal(cppRobot.mb.predecessors().begin(), cppRobot.mb.predecessors().end(),
                         strRobot.mb.predecessors().begin()));
  ASSERT_TRUE(
      std::equal(cppRobot.mb.successors().begin(), cppRobot.mb.successors().end(), strRobot.mb.successors().begin()));
  ASSERT_TRUE(std::equal(cppRobot.mb.parents().begin(), cppRobot.mb.parents().end(), strRobot.mb.parents().begin()));
  ASSERT_TRUE(
      std::equal(cppRobot.mb.transforms().begin(), cppRobot.mb.transforms().end(), strRobot.mb.transforms().begin()));

  ASSERT_TRUE(std::equal(cppRobot.limits.lower.begin(), cppRobot.limits.lower.end(), strRobot.limits.lower.begin()));
  ASSERT_TRUE(std::equal(cppRobot.limits.upper.begin(), cppRobot.limits.upper.end(), strRobot.limits.upper.begin()));
  ASSERT_TRUE(
      std::equal(cppRobot.limits.velocity.begin(), cppRobot.limits.velocity.end(), strRobot.limits.velocity.begin()));
  ASSERT_TRUE(std::equal(cppRobot.limits.torque.begin(), cppRobot.limits.torque.end(), strRobot.limits.torque.begin()));

  for(int i = 0; i < cppRobot.mb.nrBodies(); ++i)
  {
    const auto & b1 = cppRobot.mb.body(i);
    const auto & b2 = strRobot.mb.body(i);

    EXPECT_EQ(b1.name(), b2.name());

    EXPECT_EQ(b1.inertia().mass(), b2.inertia().mass());
    EXPECT_EQ(b1.inertia().momentum(), b2.inertia().momentum());
    EXPECT_NEAR((b1.inertia().inertia() - b2.inertia().inertia()).norm(), 0.0, TOL);
  }

  for(int i = 0; i < cppRobot.mb.nrJoints(); ++i)
  {
    const auto & j1 = cppRobot.mb.joint(i);
    const auto & j2 = strRobot.mb.joint(i);

    EXPECT_EQ(j1.name(), j2.name());
    EXPECT_EQ(j1.type(), j2.type());
    EXPECT_EQ(j1.direction(), j2.direction());
    EXPECT_EQ(j1.motionSubspace(), j2.motionSubspace());
  }
}

TEST(YAMLParserTest, visualTest)
{
  auto cppRobot = createRobot();
  auto strRobot = rbd::parsers::from_yaml(XYZSarmYaml);
  const auto & cpp_geometries = cppRobot.visual;
  const auto & str_geometries = strRobot.visual;

  EXPECT_EQ(str_geometries.size(), cpp_geometries.size());
  for(const auto & g : str_geometries)
  {
    EXPECT_EQ(g.second.size(), cpp_geometries.at(g.first).size());
  }

  for(const auto & body : cppRobot.mb.bodies())
  {
    EXPECT_TRUE(std::equal(strRobot.visual[body.name()].begin(), strRobot.visual[body.name()].end(),
                           cppRobot.visual[body.name()].begin()));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*
 * Copyright 2012-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// google test
#include <gtest/gtest.h>

// RBDyn YAML parser
#include <RBDyn/parsers/yaml.h>

#include <iostream>

// Test utilties
#include "ParsersTestUtils.h"

TEST(YAMLOutputTest, outputTest)
{
  auto strRobot = rbd::parsers::from_yaml(XYZSarmYaml);
  auto yaml = rbd::parsers::to_yaml(strRobot);
  std::cout << yaml << std::endl;
  auto strRobot2 = rbd::parsers::from_yaml(yaml);

  EXPECT_EQ(strRobot.mb.nrBodies(), strRobot2.mb.nrBodies());
  EXPECT_EQ(strRobot.mb.nrJoints(), strRobot2.mb.nrJoints());
  EXPECT_EQ(strRobot.mb.nrParams(), strRobot2.mb.nrParams());
  EXPECT_EQ(strRobot.mb.nrDof(), strRobot2.mb.nrDof());

  EXPECT_TRUE(std::equal(strRobot.mb.predecessors().begin(), strRobot.mb.predecessors().end(),
                         strRobot2.mb.predecessors().begin()));
  EXPECT_TRUE(
      std::equal(strRobot.mb.successors().begin(), strRobot.mb.successors().end(), strRobot2.mb.successors().begin()));
  EXPECT_TRUE(std::equal(strRobot.mb.parents().begin(), strRobot.mb.parents().end(), strRobot2.mb.parents().begin()));
  EXPECT_TRUE(
      std::equal(strRobot.mb.transforms().begin(), strRobot.mb.transforms().end(), strRobot2.mb.transforms().begin()));

  EXPECT_TRUE(std::equal(strRobot.limits.lower.begin(), strRobot.limits.lower.end(), strRobot2.limits.lower.begin()));
  EXPECT_TRUE(std::equal(strRobot.limits.upper.begin(), strRobot.limits.upper.end(), strRobot2.limits.upper.begin()));
  EXPECT_TRUE(
      std::equal(strRobot.limits.velocity.begin(), strRobot.limits.velocity.end(), strRobot2.limits.velocity.begin()));
  EXPECT_TRUE(
      std::equal(strRobot.limits.torque.begin(), strRobot.limits.torque.end(), strRobot2.limits.torque.begin()));

  const auto & str_visuals = strRobot.visual;
  const auto & str2_visuals = strRobot2.visual;
  EXPECT_EQ(str2_visuals.size(), str_visuals.size());
  for(const auto & g : str2_visuals)
  {
    EXPECT_EQ(g.second.size(), str_visuals.at(g.first).size());
  }

  const auto & str_collision = strRobot.collision;
  const auto & str2_collision = strRobot2.collision;
  EXPECT_EQ(str2_collision.size(), str_collision.size());
  for(const auto & g : str2_collision)
  {
    EXPECT_EQ(g.second.size(), str_collision.at(g.first).size());
  }

  for(const auto & body : strRobot.mb.bodies())
  {
    EXPECT_TRUE(std::equal(strRobot.visual[body.name()].begin(), strRobot.visual[body.name()].end(),
                           strRobot2.visual[body.name()].begin()));
    EXPECT_TRUE(std::equal(strRobot.collision[body.name()].begin(), strRobot.collision[body.name()].end(),
                           strRobot2.collision[body.name()].begin()));
  }

  for(int i = 0; i < strRobot.mb.nrBodies(); ++i)
  {
    const auto & b1 = strRobot.mb.body(i);
    const auto & b2 = strRobot2.mb.body(i);

    EXPECT_EQ(b1.name(), b2.name());

    EXPECT_EQ(b1.inertia().mass(), b2.inertia().mass());
    EXPECT_EQ(b1.inertia().momentum(), b2.inertia().momentum());
    EXPECT_NEAR((b1.inertia().inertia() - b2.inertia().inertia()).norm(), 0.0, TOL);
  }

  for(int i = 0; i < strRobot.mb.nrJoints(); ++i)
  {
    const auto & j1 = strRobot.mb.joint(i);
    const auto & j2 = strRobot2.mb.joint(i);

    EXPECT_EQ(j1.name(), j2.name());
    EXPECT_EQ(j1.type(), j2.type());
    EXPECT_EQ(j1.direction(), j2.direction());
    EXPECT_EQ(j1.motionSubspace(), j2.motionSubspace());
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

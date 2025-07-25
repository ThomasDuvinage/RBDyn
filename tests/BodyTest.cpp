/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// includes
// std
#include <iostream>

// google test
#include <gtest/gtest.h>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/Body.h"

TEST(BodyTest, BodyTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  double mass = 1.;
  Matrix3d I;
  I << 1., 2., 3., 2., 1., 4., 3., 4., 1.;
  Vector3d h = Vector3d::Random() * 100.;

  RBInertiad rbi(mass, h, I);

  // Test first constructor
  Body b1(rbi, "b1");

  EXPECT_EQ(b1.name(), "b1");
  EXPECT_EQ(b1.inertia().mass(), rbi.mass());
  EXPECT_EQ(b1.inertia().momentum(), rbi.momentum());
  EXPECT_EQ(b1.inertia().inertia(), rbi.inertia());

  // Test second constructor
  Body b2(mass, Vector3d::UnitX(), I, "b2");

  EXPECT_EQ(b2.name(), "b2");
  EXPECT_EQ(b2.inertia().mass(), mass);
  EXPECT_EQ(b2.inertia().momentum(), mass * Vector3d::UnitX());
  EXPECT_EQ(b2.inertia().inertia(), I);

  // Test operator==
  EXPECT_EQ(b1, b1);
  EXPECT_NE(b1, b2);

  // Test operator!=
  EXPECT_TRUE(!(b1 != b1));
  EXPECT_TRUE(b1 != b2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

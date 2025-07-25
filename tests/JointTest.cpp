/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

// includes
// std
#include <iostream>

// google test
#include "CollectionTest.h"
#include <gtest/gtest.h>

// SpaceVecAlg
#include <SpaceVecAlg/SpaceVecAlg>

// RBDyn
#include "RBDyn/Joint.h"

const double TOL = 1e-10;

namespace rbd
{

static constexpr double PI = 3.141592653589793238462643383279502884e+00;

} // namespace rbd

void testRevolute(rbd::Joint::OldType type, const Eigen::Vector3d & axis, bool forward)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  Joint j(type, forward, "rev");
  double dir = forward ? 1. : -1;

  // test motion
  Vector6d S;
  S << dir * axis, 0., 0., 0.;

  // test motion
  MotionVecd motion(S);

  // test motion
  PTransformd rot90(AngleAxisd(-rbd::PI / 2., dir * axis).matrix());

  // test accessor
  EXPECT_EQ(j.type(), Joint::Rev);
  EXPECT_EQ(j.params(), 1);
  EXPECT_EQ(j.dof(), 1);
  EXPECT_EQ(j.name(), "rev");
  EXPECT_EQ(j.motionSubspace(), S);

  // test zero
  std::vector<double> zeroP = {0.};
  std::vector<double> zeroD = {0.};
  std::vector<double> zp = j.zeroParam();
  std::vector<double> zd = j.zeroDof();

  EXPECT_TRUE(CheckEqualCollections(zp.begin(), zp.end(), zeroP.begin()));

  EXPECT_TRUE(CheckEqualCollections(zd.begin(), zd.end(), zeroD.begin()));

  EXPECT_EQ(j.pose<double>({rbd::PI / 2.}), rot90);

  EXPECT_EQ(j.motion({2.}).vector(), (2. * motion).vector());
}

void testPrismatic(rbd::Joint::OldType type, const Eigen::Vector3d & axis, bool forward)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  Joint j(type, forward, "prism");
  double dir = forward ? 1. : -1;

  // test motion
  Vector6d S;
  S << 0., 0., 0., dir * axis;

  // test motion
  MotionVecd motion(S);

  // test motion
  PTransformd trans2(Vector3d(dir * axis * 2.));

  // test accessor
  EXPECT_EQ(j.type(), Joint::Prism);
  EXPECT_EQ(j.params(), 1);
  EXPECT_EQ(j.dof(), 1);
  EXPECT_EQ(j.name(), "prism");
  EXPECT_EQ(j.motionSubspace(), S);

  // test zero
  std::vector<double> zeroP = {0.};
  std::vector<double> zeroD = {0.};
  std::vector<double> zp = j.zeroParam();
  std::vector<double> zd = j.zeroDof();
  EXPECT_TRUE(CheckEqualCollections(zp.begin(), zp.end(), zeroP.begin()));

  EXPECT_TRUE(CheckEqualCollections(zd.begin(), zd.end(), zeroD.begin()));

  // test motion
  EXPECT_EQ(j.pose<double>({2.}), trans2);

  // test motion
  EXPECT_EQ(j.motion({2.}).vector(), (2. * motion).vector());
}

TEST(FILENAME, JointTest)
{
  using namespace rbd;

  // test operator==
  Joint j1(Joint::RevX, true, "j1");
  Joint j2(Joint::RevX, false, "j2");

  EXPECT_EQ(j1, j1);
  EXPECT_NE(j1, j2);

  // Test operator!=
  EXPECT_TRUE(!(j1 != j1));
  EXPECT_TRUE(j1 != j2);

  // Test direction
  EXPECT_EQ(j1.direction(), 1.);
  EXPECT_EQ(j2.direction(), -1.);

  // Test forward (getter)
  EXPECT_EQ(j1.forward(), true);
  EXPECT_EQ(j2.forward(), false);

  // Test forward (setter)
  j1.forward(false);
  EXPECT_EQ(j1.direction(), -1.);
  EXPECT_EQ(j1.forward(), false);

  // sPose
  EXPECT_THROW(j1.sPose({0., 0.}), std::domain_error);
  EXPECT_THROW(j1.sPose({}), std::domain_error);
  EXPECT_NO_THROW(j1.sPose({0.}));
  EXPECT_EQ(j1.sPose({0.}), j1.pose<double>({0.}));

  // sMotion
  EXPECT_THROW(j1.sMotion({0., 0.}), std::domain_error);
  EXPECT_THROW(j1.sMotion({}), std::domain_error);
  EXPECT_NO_THROW(j1.sMotion({0.}));
  EXPECT_EQ(j1.sMotion({0.}), j1.motion({0.}));
}

TEST(FILENAME, RevXTest)
{
  using namespace Eigen;
  using namespace rbd;
  testRevolute(Joint::RevX, Vector3d::UnitX(), true);
  testRevolute(Joint::RevX, Vector3d::UnitX(), false);
}

TEST(FILENAME, RevYTest)
{
  using namespace Eigen;
  using namespace rbd;
  testRevolute(Joint::RevY, Vector3d::UnitY(), true);
  testRevolute(Joint::RevY, Vector3d::UnitY(), false);
}

TEST(FILENAME, RevZTest)
{
  using namespace Eigen;
  using namespace rbd;
  testRevolute(Joint::RevZ, Vector3d::UnitZ(), true);
  testRevolute(Joint::RevZ, Vector3d::UnitZ(), false);
}

TEST(FILENAME, PrismXTest)
{
  using namespace Eigen;
  using namespace rbd;
  testPrismatic(Joint::PrismX, Vector3d::UnitX(), true);
  testPrismatic(Joint::PrismX, Vector3d::UnitX(), false);
}

TEST(FILENAME, PrismYTest)
{
  using namespace Eigen;
  using namespace rbd;
  testPrismatic(Joint::PrismY, Vector3d::UnitY(), true);
  testPrismatic(Joint::PrismY, Vector3d::UnitY(), false);
}

TEST(FILENAME, PrismZTest)
{
  using namespace Eigen;
  using namespace rbd;
  testPrismatic(Joint::PrismZ, Vector3d::UnitZ(), true);
  testPrismatic(Joint::PrismZ, Vector3d::UnitZ(), false);
}

TEST(FILENAME, SphericalTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  Joint j(Joint::Spherical, true, "sphere");

  // subspace data
  MatrixXd S = MatrixXd::Zero(6, 3);
  S.block(0, 0, 3, 3).setIdentity();

  // pose data
  double rotX = rbd::PI / 2.;
  double rotY = rbd::PI / 4.;
  double rotZ = rbd::PI;

  Quaterniond quat =
      AngleAxisd(rotX, Vector3d::UnitX()) * AngleAxisd(rotY, Vector3d::UnitY()) * AngleAxisd(rotZ, Vector3d::UnitZ());

  std::vector<double> q = {quat.w(), quat.x(), quat.y(), quat.z()};

  PTransformd rot(quat.inverse());

  // motion data
  std::vector<double> alpha;
  Vector3d alphaE = Vector3d::Random();
  for(int i = 0; i < 3; ++i) alpha.push_back(alphaE(i));

  // test accessor
  EXPECT_EQ(j.type(), Joint::Spherical);
  EXPECT_EQ(j.params(), 4);
  EXPECT_EQ(j.dof(), 3);
  EXPECT_EQ(j.name(), "sphere");
  EXPECT_EQ(j.motionSubspace(), S);

  // test zero
  std::vector<double> zeroP = {1., 0., 0., 0.};
  std::vector<double> zeroD = {0., 0., 0.};
  std::vector<double> zp = j.zeroParam();
  std::vector<double> zd = j.zeroDof();
  EXPECT_TRUE(CheckEqualCollections(zp.begin(), zp.end(), zeroP.begin()));

  EXPECT_TRUE(CheckEqualCollections(zd.begin(), zd.end(), zeroD.begin()));

  // test pose
#ifdef __i386__
  EXPECT_NEAR((j.pose(q).matrix() - rot.matrix()).array().abs().sum(), 0.0, TOL);
#else
  EXPECT_EQ(j.pose(q), rot);
#endif

  // test motion
  EXPECT_EQ(j.motion(alpha).vector(), S * alphaE);

  // test inverse polarity
  j.forward(false);
  EXPECT_EQ(j.motionSubspace(), -S);
#ifdef __i386__
  EXPECT_NEAR((j.pose(q).matrix() - rot.inv().matrix()).array().abs().sum(), 0.0, TOL);
#else
  EXPECT_EQ(j.pose(q), rot.inv());
#endif
  EXPECT_EQ(j.motion(alpha).vector(), -S * alphaE);
}

TEST(FILENAME, PlanarTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  Joint j(Joint::Planar, true, "planar");

  // subspace data
  MatrixXd S = MatrixXd::Zero(6, 3);
  S.block(2, 0, 3, 3).setIdentity();

  // pose data
  double rotZ = rbd::PI / 2.;
  double transX = 4.;
  double transY = 0.3;

  sva::PTransformd trans(RotZ(rotZ), RotZ(rotZ).transpose() * Vector3d(transX, transY, 0.));

  std::vector<double> q = {rotZ, transX, transY};

  // motion data
  std::vector<double> alpha;
  Vector3d alphaE = Vector3d::Random();
  for(int i = 0; i < 3; ++i) alpha.push_back(alphaE(i));

  // test accessor
  EXPECT_EQ(j.type(), Joint::Planar);
  EXPECT_EQ(j.params(), 3);
  EXPECT_EQ(j.dof(), 3);
  EXPECT_EQ(j.name(), "planar");
  EXPECT_EQ(j.motionSubspace(), S);

  // test zero
  std::vector<double> zeroP = {0., 0., 0.};
  std::vector<double> zeroD = {0., 0., 0.};
  std::vector<double> zp = j.zeroParam();
  std::vector<double> zd = j.zeroDof();
  EXPECT_TRUE(CheckEqualCollections(zp.begin(), zp.end(), zeroP.begin()));

  EXPECT_TRUE(CheckEqualCollections(zd.begin(), zd.end(), zeroD.begin()));

  // test pose
  EXPECT_EQ(j.pose(q), trans);

  // test motion
  EXPECT_EQ(j.motion(alpha).vector(), S * alphaE);

  // test inverse polarity
  j.forward(false);
  EXPECT_EQ(j.motionSubspace(), -S);
  EXPECT_EQ(j.pose(q), trans.inv());
  EXPECT_EQ(j.motion(alpha).vector(), -S * alphaE);
}

TEST(FILENAME, CylindricalTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  Vector3d axis(Vector3d::Random().normalized());
  Joint j(Joint::Cylindrical, axis, true, "cylindrical");

  // subspace data
  MatrixXd S = MatrixXd::Zero(6, 2);
  S.col(0).head<3>() = axis;
  S.col(1).tail<3>() = axis;

  // pose data
  double rot = rbd::PI / 2.;
  double trans = 5.;

  sva::PTransformd X(AngleAxisd(-rot, axis).matrix(), axis * trans);

  std::vector<double> q = {rot, trans};

  // motion data
  std::vector<double> alpha;
  Vector2d alphaE = Vector2d::Random();
  for(int i = 0; i < 2; ++i) alpha.push_back(alphaE(i));

  // test accessor
  EXPECT_EQ(j.type(), Joint::Cylindrical);
  EXPECT_EQ(j.params(), 2);
  EXPECT_EQ(j.dof(), 2);
  EXPECT_EQ(j.name(), "cylindrical");
  EXPECT_EQ(j.motionSubspace(), S);

  // test zero
  std::vector<double> zeroP = {0., 0.};
  std::vector<double> zeroD = {0., 0.};
  std::vector<double> zp = j.zeroParam();
  std::vector<double> zd = j.zeroDof();
  EXPECT_TRUE(CheckEqualCollections(zp.begin(), zp.end(), zeroP.begin()));

  EXPECT_TRUE(CheckEqualCollections(zd.begin(), zd.end(), zeroD.begin()));

  // test pose
  EXPECT_EQ(j.pose(q), X);

  // test motion
  EXPECT_EQ(j.motion(alpha).vector(), S * alphaE);

  // test inverse polarity
  j.forward(false);
  EXPECT_EQ(j.motionSubspace(), -S);
  EXPECT_NEAR((j.pose(q).matrix() - X.inv().matrix()).norm(), 0.0, TOL);
  EXPECT_EQ(j.motion(alpha).vector(), -S * alphaE);
}

TEST(FILENAME, FreeTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  Joint j(Joint::Free, true, "free");

  // subspace data
  MatrixXd S = MatrixXd::Identity(6, 6);

  // pose data
  double rotX = rbd::PI / 2.;
  double rotY = rbd::PI / 4.;
  double rotZ = rbd::PI;

  Quaterniond quat =
      AngleAxisd(rotX, Vector3d::UnitX()) * AngleAxisd(rotY, Vector3d::UnitY()) * AngleAxisd(rotZ, Vector3d::UnitZ());
  Vector3d trans = Vector3d::Random();

  std::vector<double> q = {quat.w(), quat.x(), quat.y(), quat.z(), trans.x(), trans.y(), trans.z()};

  PTransformd rot(quat.matrix().transpose(), trans);

  // motion data
  std::vector<double> alpha;
  Vector6d alphaE = Vector6d::Random();
  for(int i = 0; i < 6; ++i) alpha.push_back(alphaE(i));

  // test accessor
  EXPECT_EQ(j.type(), Joint::Free);
  EXPECT_EQ(j.params(), 7);
  EXPECT_EQ(j.dof(), 6);
  EXPECT_EQ(j.name(), "free");
  EXPECT_EQ(j.motionSubspace(), S);

  // test zero
  std::vector<double> zeroP = {1., 0., 0., 0., 0., 0., 0.};
  std::vector<double> zeroD = {0., 0., 0., 0., 0., 0.};
  std::vector<double> zp = j.zeroParam();
  std::vector<double> zd = j.zeroDof();
  EXPECT_TRUE(CheckEqualCollections(zp.begin(), zp.end(), zeroP.begin()));

  EXPECT_TRUE(CheckEqualCollections(zd.begin(), zd.end(), zeroD.begin()));

  // test pose
  EXPECT_NEAR((j.pose(q).matrix() - rot.matrix()).norm(), 0.0, TOL);

  // test motion
  EXPECT_EQ(j.motion(alpha).vector(), S * alphaE);

  // test inverse polarity
  j.forward(false);
  EXPECT_EQ(j.motionSubspace(), -S);
  EXPECT_NEAR((j.pose(q).matrix() - rot.inv().matrix()).norm(), 0.0, TOL);
  EXPECT_EQ(j.motion(alpha).vector(), -S * alphaE);
}

TEST(FILENAME, FixedTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  Joint j(Joint::Fixed, true, "fixed");

  // subspace data
  MatrixXd S = MatrixXd::Zero(6, 0);

  // test accessor
  EXPECT_EQ(j.type(), Joint::Fixed);
  EXPECT_EQ(j.params(), 0);
  EXPECT_EQ(j.dof(), 0);
  EXPECT_EQ(j.name(), "fixed");
  EXPECT_EQ(j.motionSubspace(), S);

  // test pose
  EXPECT_EQ(j.pose<double>({}), PTransformd::Identity());

  // test motion
  EXPECT_EQ(j.motion({}).vector(), Vector6d::Zero());
}

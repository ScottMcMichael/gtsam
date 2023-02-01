/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSerializationNonlinear.cpp
 * @brief
 * @author Richard Roberts
 * @date Feb 7, 2012
 */

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/navigation/GPSFactor.h>

#include <gtsam/base/serializationTestHelpers.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;


/* ************************************************************************* */
// Create GUIDs for Noisemodels
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base , "gtsam_noiseModel_mEstimator_Base")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null , "gtsam_noiseModel_mEstimator_Null")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair , "gtsam_noiseModel_mEstimator_Fair")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber, "gtsam_noiseModel_mEstimator_Huber")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,"gtsam_noiseModel_Isotropic")
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust")
BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel")
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal")

/* ************************************************************************* */
// Create GUIDs for factors
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Pose3>, "gtsam::PriorFactor<gtsam::Pose3>")
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Pose3>, "gtsam::BetweenFactor<gtsam::Pose3>")
BOOST_CLASS_EXPORT_GUID(gtsam::GPSFactor, "gtsam::GPSFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::PoseRotationPrior<gtsam::Pose3>, "gtsam::PoseRotationPrior<gtsam::Pose3>");
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor")
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsam::HessianFactor")
BOOST_CLASS_EXPORT_GUID(gtsam::GaussianConditional , "gtsam::GaussianConditional")


/* ************************************************************************* */
// Export all classes derived from Value
GTSAM_VALUE_EXPORT(gtsam::Cal3_S2)
GTSAM_VALUE_EXPORT(gtsam::Cal3Bundler)
GTSAM_VALUE_EXPORT(gtsam::Point3)
GTSAM_VALUE_EXPORT(gtsam::Pose3)
GTSAM_VALUE_EXPORT(gtsam::Rot3)
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3_S2>)
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3DS2>)
GTSAM_VALUE_EXPORT(gtsam::PinholeCamera<Cal3Bundler>)

namespace detail {
template<class T> struct pack {
 typedef T type;
};
}

/* ************************************************************************* */
typedef PinholeCamera<Cal3_S2>        PinholeCal3S2;
typedef PinholeCamera<Cal3DS2>        PinholeCal3DS2;
typedef PinholeCamera<Cal3Bundler>    PinholeCal3Bundler;

/* ************************************************************************* */
static Point3 pt3(1.0, 2.0, 3.0);
static Rot3 rt3 = Rot3::RzRyRx(1.0, 3.0, 2.0);
static Pose3 pose3(rt3, pt3);

static Cal3_S2 cal1(1.0, 2.0, 0.3, 0.1, 0.5);
static Cal3DS2 cal2(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
static Cal3Bundler cal3(1.0, 2.0, 3.0);

TEST (Serialization, TemplatedValues) {
  EXPECT(equalsObj(pt3));
  GenericValue<Point3> chv1(pt3);
  EXPECT(equalsObj(chv1));
  PinholeCal3S2 pc(pose3,cal1);
  EXPECT(equalsObj(pc));

  Values values;
  values.insert(1,pt3);

  EXPECT(equalsObj(values));
  values.insert(Symbol('a',0),  PinholeCal3S2(pose3, cal1));
  values.insert(Symbol('s',5), PinholeCal3DS2(pose3, cal2));
  values.insert(Symbol('d',47), PinholeCal3Bundler(pose3, cal3));
  values.insert(Symbol('a',5),  PinholeCal3S2(pose3, cal1));
  EXPECT(equalsObj(values));
  EXPECT(equalsXML(values));
  EXPECT(equalsBinary(values));
}

/**
 * Test deserializing from a known serialization generated by code from commit
 * 0af17f438f62f4788f3a572ecd36d06d224fd1e1 (>4.2a7)
 * We only test that deserialization matches since
 *  (1) that's the main backward compatibility requirement and
 *  (2) serialized string depends on boost version
 * Also note: we don't run this test when quaternions or TBB are enabled since
 * serialization structures are different and the serialized strings/xml are
 * hard-coded in this test.
 */
TEST(Serialization, NoiseModelFactor1_backwards_compatibility) {
  PriorFactor<Pose3> factor(
      12345, Pose3(Rot3::RzRyRx(Vector3(1., 2., 3.)), Point3(4., 5., 6.)),
      noiseModel::Unit::Create(6));

  // roundtrip (sanity check)
  PriorFactor<Pose3> factor_deserialized_str_2 = PriorFactor<Pose3>();
  roundtrip(factor, factor_deserialized_str_2);
  EXPECT(assert_equal(factor, factor_deserialized_str_2));

#if !defined(GTSAM_USE_QUATERNIONS) && !defined(GTSAM_USE_TBB)
  // Deserialize string
  std::string serialized_str =
      "22 serialization::archive 15 1 0\n"
      "0 0 0 0 0 0 0 1 0 12345 0 1 6 21 gtsam_noiseModel_Unit 1 0\n"
      "1 1 0\n"
      "2 1 0\n"
      "3 0 0 0 0 6 0 1 0 0 0 6 1.00000000000000000e+00 1.00000000000000000e+00 "
      "1.00000000000000000e+00 1.00000000000000000e+00 1.00000000000000000e+00 "
      "1.00000000000000000e+00 6 1.00000000000000000e+00 "
      "1.00000000000000000e+00 1.00000000000000000e+00 1.00000000000000000e+00 "
      "1.00000000000000000e+00 1.00000000000000000e+00 1.00000000000000000e+00 "
      "1.00000000000000000e+00 0 0 0 0 4.11982245665682978e-01 "
      "-8.33737651774156818e-01 -3.67630462924899259e-01 "
      "-5.87266449276209815e-02 -4.26917621276207360e-01 "
      "9.02381585483330806e-01 -9.09297426825681709e-01 "
      "-3.50175488374014632e-01 -2.24845095366152908e-01 0 0 "
      "4.00000000000000000e+00 5.00000000000000000e+00 "
      "6.00000000000000000e+00\n";

  PriorFactor<Pose3> factor_deserialized_str = PriorFactor<Pose3>();
  deserializeFromString(serialized_str, factor_deserialized_str);
  EXPECT(assert_equal(factor, factor_deserialized_str));

  // Deserialize XML
  PriorFactor<Pose3> factor_deserialized_xml = PriorFactor<Pose3>();
  deserializeFromXMLFile(GTSAM_SOURCE_TREE_DATASET_DIR
                         "/../../gtsam/nonlinear/tests/priorFactor.xml",
                         factor_deserialized_xml);
  EXPECT(assert_equal(factor, factor_deserialized_xml));
#endif
}

TEST(Serialization, ISAM2) {
  gtsam::ISAM2Params parameters;
  gtsam::ISAM2 solver(parameters);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initialValues;
  initialValues.clear();

  gtsam::Vector6 temp6;
  for (int i = 0; i < 6; ++i) {
    temp6[i] = 0.0001;
  }
  gtsam::noiseModel::Diagonal::shared_ptr noiseModel = gtsam::noiseModel::Diagonal::Sigmas(temp6);

  gtsam::Pose3 pose0(gtsam::Rot3(), gtsam::Point3(0, 0, 0));
  gtsam::Symbol symbol0('x', 0);
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(symbol0, pose0, noiseModel));
  initialValues.insert(symbol0, pose0);

  solver.update(graph, initialValues,
                gtsam::FastVector<gtsam::FactorIndex>());

  std::string binaryPath = "saved_solver.dat";
  try {
    std::ofstream outputStream(binaryPath);
    boost::archive::binary_oarchive outputArchive(outputStream);
    outputArchive << solver;
  } catch(...) {
    EXPECT(false);
  }

  gtsam::ISAM2 solverFromDisk;
  try {
    std::ifstream ifs(binaryPath);
    boost::archive::binary_iarchive inputArchive(ifs);
    inputArchive >> solverFromDisk;
  } catch(...) {
    EXPECT(false);
  }

  gtsam::Pose3 p1, p2;
  try {
    p1 = solver.calculateEstimate<gtsam::Pose3>(symbol0);
  } catch(std::exception &e) {
    EXPECT(false);
  }

  try {
    p2 = solverFromDisk.calculateEstimate<gtsam::Pose3>(symbol0);
  } catch(std::exception &e) {
    EXPECT(false);
  }
  EXPECT(assert_equal(p1, p2));
}



TEST(Serialization, largeSerialization) {

  const std::string gtsamBinaryPath = "large_factor_graph.dat";

  gtsam::NonlinearFactorGraph isamGraph;
  gtsam::Values isamInitialValues;
  gtsam::ISAM2 isamSolver;
  gtsam::FastVector<size_t> factorsToRemove;
  size_t symbolCounter = 0;
  
  // Load graph from data file
  //std::ifstream inputStream(gtsamBinaryPath);
  //boost::archive::binary_iarchive inputArchive(inputStream);
  //inputArchive >> isamGraph
  //             >> isamSolver
  //             >> symbolCounter;


  gtsam::Vector6 temp6;
  for (int i = 0; i < 10; ++i) {
    temp6[i] = 0.1;
  }
  gtsam::noiseModel::Diagonal::shared_ptr noiseModel = gtsam::noiseModel::Diagonal::Sigmas(temp6);
  gtsam::Point3 p(0.3, -0.1, 0.01);
  gtsam::Quaternion q(1.0, 0.0, 0.0, 0.0);
  gtsam::Pose3 transform(gtsam::Rot3(q), p);

  // Generate a new graph
  isamGraph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', 0), gtsam::Pose3(), noiseModel));
  isamInitialValues.insert_or_assign(gtsam::Symbol('x', 0), gtsam::Pose3());
  ++symbolCounter;

  for (int i=0; i<5000; ++i) {
    size_t lastSymbolIndex = symbolCounter - 1;
    size_t nextSymbolIndex = symbolCounter;

    gtsam::Symbol lastSymbol(gtsam::Symbol('x', lastSymbolIndex));
    gtsam::Symbol nextSymbol(gtsam::Symbol('x', nextSymbolIndex));
    isamGraph.add(gtsam::BetweenFactor<gtsam::Pose3>(lastSymbol, nextSymbol, transform, noiseModel));
    isamInitialValues.insert_or_assign(nextSymbol, transform);

    isamSolver.update(isamGraph, isamInitialValues, factorsToRemove);

    ++symbolCounter;
    isamGraph.resize(0);
    isamInitialValues.clear();
    factorsToRemove.clear();
    
    if (i > 4400) {
      std::string outputFolder = "./";
      std::stringstream ss;
      ss << outputFolder << "/gt" << i << ".bin";
      std::cout << "Write: " << ss.str() << std::endl;
      std::ofstream outputStream(ss.str());
      boost::archive::binary_oarchive outputArchive(outputStream);
      outputArchive << isamSolver;
    }
  }
  EXPECT(true);
}



/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

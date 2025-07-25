/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef TESTS_DATA_DIR
const static std::filesystem::path tests_data_dir(TESTS_DATA_DIR);
#else
const static std::filesystem::path tests_data_dir;
#endif //! TESTS_DATA_DIR

namespace rbd
{
struct TestSuiteConfiguration
{
  TestSuiteConfiguration() {}

  ~TestSuiteConfiguration() {}
};

std::string retrievePattern(const std::string & testName)
{
  std::string patternFilename = std::string(TESTS_DATA_DIR) + "/" + testName + ".stdout";

  std::ifstream file(patternFilename);
  if(!file)
  {
    throw std::runtime_error("Could not open pattern file: " + patternFilename);
  }

  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
} // end of namespace rbd

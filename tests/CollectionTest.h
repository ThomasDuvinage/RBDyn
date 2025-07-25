#pragma once

#include <gtest/gtest.h>

template<typename LeftIter, typename RightIter>
::testing::AssertionResult CheckEqualCollections(LeftIter left_begin, LeftIter left_end, RightIter right_begin)
{
  bool equal(true);
  std::string message;
  std::size_t index(0);
  while(left_begin != left_end)
  {
    if(*left_begin++ != *right_begin++)
    {
      equal = false;
      message += "\n\tMismatch at index " + std::to_string(index);
    }
    ++index;
  }
  if(message.size()) message += "\n\t";
  return equal ? ::testing::AssertionSuccess() : ::testing::AssertionFailure() << message;
}

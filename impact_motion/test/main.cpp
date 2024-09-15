#include "gtest/gtest.h"

int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  // execute all tests defined in CMakeLists.txt
  return RUN_ALL_TESTS();
}

TEST(TestSuite, testCase) {
  EXPECT_TRUE(true);
}
// Copyright [year] <Copyright Owner>
#include <gtest/gtest.h>

TEST(offboard_control, basic_test) {
  // Basic test to ensure the test framework is working
  EXPECT_EQ(1, 1);
  ASSERT_EQ(1, 1);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

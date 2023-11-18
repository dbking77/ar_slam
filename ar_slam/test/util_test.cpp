#include <gtest/gtest.h>

#include <string>

#include "ar_slam/ar_slam_util.hpp"

TEST(ar_slam_util, filename_no_ext)
{
  /*
   *   file.jpg          -> file
   *   /path/to/file.jpg -> file
   *   ../../file.jpg    -> file
   *   ../file           -> file
   *   ../file.1.jpg     -> file.1
   */
  ASSERT_EQ(filename_no_ext("file.jpg"), "file");
  ASSERT_EQ(filename_no_ext("/path/to/file.jpg"), "file");
  ASSERT_EQ(filename_no_ext("../../file.jpg"), "file");
  ASSERT_EQ(filename_no_ext("../file"), "file");
  ASSERT_EQ(filename_no_ext("../file.1.jpg"), "file.1");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <au_core/loader_util.hpp>
#include <gtest/gtest.h>

using namespace au_core;

TEST(LoaderUtilTest, loadAvailableTopic) {
  try {
    ASSERT_STREQ(au_core::load_topic("/topic/test").c_str(), "/test");
  } catch (std::exception& e) {
    FAIL();
  }
}

TEST(LoaderUtilTest, loadUnavailableTopic) {
  try {
    std::string tmp = au_core::load_topic("/test_topic");
    FAIL();  // should throw exception before getting here
  } catch (std::exception& e) {
    ASSERT_STREQ(e.what(), "invalid topic key: /test_topic");
  }
}

TEST(LoaderUtilTest, loadAvailableFrame) {
  try {
    ASSERT_STREQ(au_core::load_frame("test").c_str(), "test_frame");
    ASSERT_STREQ(au_core::load_frame("test").c_str(), "test_frame");
  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
    FAIL();
  }
}

TEST(LoaderUtilTest, loadUnavailableFrame) {
  try {
    std::string tmp = au_core::load_frame("test_again");
    FAIL();  // should throw exception before getting here
  } catch (std::exception& e) {
    ASSERT_STREQ(e.what(), "invalid frame key: test_again");
  }
}

// Run all the tests
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

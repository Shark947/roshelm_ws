#include <gtest/gtest.h>

#include "bt_executor/utils/utils.hpp"

namespace bt_executor
{
namespace
{

TEST(UtilsTest, NormalizeAngle360)
{
  EXPECT_DOUBLE_EQ(normalizeAngle360(370.0), 10.0);
  EXPECT_DOUBLE_EQ(normalizeAngle360(-30.0), 330.0);
}

TEST(UtilsTest, ParseKeyValueList)
{
  const auto data = parseKeyValueList("pwt=50,heading=90");
  ASSERT_EQ(data.size(), 2u);
  EXPECT_EQ(data.at("pwt"), "50");
  EXPECT_EQ(data.at("heading"), "90");
}

TEST(UtilsTest, ParseDockDepthUpdate)
{
  const auto depth = parseDockDepthUpdate("depth=79.5");
  ASSERT_TRUE(depth.has_value());
  EXPECT_DOUBLE_EQ(*depth, 79.5);
}

}  // namespace
}  // namespace bt_executor

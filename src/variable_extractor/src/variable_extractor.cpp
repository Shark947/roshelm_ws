// pluginlib 导出宏（class name 要与 上面 的 C++ 全名 一致）
#include <pluginlib/class_list_macros.h>
#include "variable_extractor/VariableExtractorInterface.hpp"
#include "variable_extractor/DefaultVariableExtractor.hpp"

PLUGINLIB_EXPORT_CLASS(
  variable_extractor::DefaultVariableExtractor,
  variable_extractor::VariableExtractorInterface
)
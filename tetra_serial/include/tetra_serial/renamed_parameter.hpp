#ifndef TETRA_SERIAL__RENAMED_PARAMETER_HPP_
#define TETRA_SERIAL__RENAMED_PARAMETER_HPP_

// 이름을 바꾼 드라이버 파라미터(예전 m_b* 헝가리안 이름 → 소문자 이름)를 읽는다. 구동·전원 드라이버 공용.
// tetra_serial 라이브러리 자체는 rclcpp 를 쓰지 않는다. 이 헤더는 rclcpp 노드인 드라이버만 include 한다.

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace tetra_serial
{

/**
 * 새 이름이 우선이고, 예전 이름만 주어졌으면 경고를 남기고 그 값을 쓴다.
 * 예전 런치 파일·다른 PC 의 설정이 조용히 기본값으로 돌아가지 않게 하려는 것이다.
 */
inline bool DeclareRenamedBool(
  rclcpp::Node & node, const std::string & name, const std::string & legacy_name, bool default_value)
{
  node.declare_parameter(name, default_value);
  node.declare_parameter(legacy_name, default_value);
  const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  if (overrides.count(name) == 0 && overrides.count(legacy_name) != 0) {
    RCLCPP_WARN(
      node.get_logger(), "parameter '%s' is deprecated, use '%s'", legacy_name.c_str(), name.c_str());
    return node.get_parameter(legacy_name).as_bool();
  }
  return node.get_parameter(name).as_bool();
}

}  // namespace tetra_serial

#endif  // TETRA_SERIAL__RENAMED_PARAMETER_HPP_

#include <rclcpp/rclcpp.hpp>

struct Params {
  std::string my_string;
  int my_number;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("minimal_param_node");
  auto const _ = node->add_on_set_parameters_callback(
    [](std::vector<rclcpp::Parameter> const& params)
    {
      auto result = rcl_interfaces::msg::SetParametersResult{};
      result.successful = true;

      for (auto const& param : params) {
        if(param.get_name() == "my_string") {
          auto const value = param.get_value<std::string>();
          auto const valid = std::vector<std::string>{"world", "base", "home"};

          if (std::find(valid.cbegin(), valid.cend(), value) == valid.end()) {
            result.successful = false;
            result.reason = std::string("my_string: {")
              .append(value)
              .append("} not one of: [world, base, home]");
            return result;
          }
        }

        if(param.get_name() == "my_number") {
          auto const value = param.get_value<int>();

          if (value % 23 != 0) {
            result.successful = false;
            result.reason = std::string("my_number: {")
              .append(std::to_string(value))
              .append("} must be a multiple of 23");
            return result;
          }
        }
      }
      return result;
    });

  auto param_desc  = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "Mine!";
  param_desc.additional_constraints  = "One of [world, base, home]";
  node->declare_parameter("my_string", "world", param_desc);

  param_desc  = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "What is the answer to the universe?";
  param_desc.additional_constraints  = "A multiple of 23";
  node->declare_parameter("my_number", 0, param_desc);

  auto params = Params{};
  params.my_string = node->declare_parameter("my_string");
  params.my_number = node->declare_parameter("my_number");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

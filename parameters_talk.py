#!/usr/bin/env python3

import elsie
from elsie.ext import unordered_list
from elsie.boxtree.box import Box

slides = elsie.SlideDeck(width=1920, height=1080)

slides.update_style("default", elsie.TextStyle(font="Lato", align="left", size=64))
slides.update_style("code", elsie.TextStyle(size=38))
slides.set_style("link", elsie.TextStyle(color="blue"))


def logo_header_slide(parent: Box, title: str):
    parent.box(x=1570, y=40).image("picknik_logo.png")
    parent.sbox(name="header", x=0, height=140).fbox(p_left=20).text(
        title, elsie.TextStyle(bold=True)
    )
    return parent.fbox(name="content", p_left=20, p_right=20)


def image_slide(parent: Box, title: str, image_path: str):
    content = logo_header_slide(parent, title)
    content = content.fbox(horizontal=True, p_top=20, p_bottom=20)
    text_area = content.fbox(name="text_area", width="50%")
    content.sbox(name="image", width="fill").image(image_path)
    return text_area


def section_title_slide(parent: Box, title: str, subtitle: str):
    content = logo_header_slide(parent, "")
    content.sbox().text(title, elsie.TextStyle(align="right", size=240, bold=True))
    content.box().text(subtitle)


def code_slide(parent: Box, title: str, language: str, code: str):
    content = logo_header_slide(parent, title)
    code_bg = "#F6F8FA"
    box = content.box(y=0, width="100%", height="100%", p_bottom=20)
    box.rect(bg_color=code_bg, rx=20, ry=20)
    box.overlay().box(x=0, y=0, p_left=20, p_right=20, p_top=20, p_bottom=20).code(
        language, code
    )


@slides.slide(debug_boxes=False)
def title(slide):
    content = logo_header_slide(slide, "")
    content.box(width="fill").text(
        "Parameters Should\nBe Boring", elsie.TextStyle(size=160, bold=True)
    )
    content.box(width="fill").text(
        "generate_parameter_library", elsie.TextStyle(size=36, bold=True, italic=True)
    )
    content.box(width="fill", p_top=10).text("October 20, 2023")
    content.box(width="fill", p_top=180).text("Tyler Weaver")
    content.box(width="fill").text("Staff Software Engineer\ntyler@picknik.ai")


@slides.slide(debug_boxes=False)
def author(slide):
    text_area = image_slide(slide, "Tyler Weaver", "images/kart.jpg")
    lst = unordered_list(text_area)
    lst.item().text("Racing Kart Driver")
    lst.item().text("MoveIt Maintainer")
    lst.item().text("Rust Evangelist")
    lst.item().text("Docker Skeptic")


@slides.slide(debug_boxes=False)
def part_1(slide):
    section_title_slide(slide, "RCLCPP\nParameters", "Part 1")


@slides.slide(debug_boxes=False)
def innocence(slide):
    code_slide(
        slide,
        "Getting Started",
        "C++",
        """
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("minimal_param_node");
  auto my_string = node->declare_parameter("my_string", "world");
  auto my_number = node->declare_parameter("my_number", 23);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
""",
    )


@slides.slide(debug_boxes=False)
def struct_with_defaults(slide):
    code_slide(
        slide,
        "Parameter Struct",
        "C++",
        """
struct Params {
  std::string my_string = "world";
  int my_number = 23;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("minimal_param_node");
  auto params = Params{};
  params.my_string = node->declare_parameter("my_string", params.my_string);
  params.my_number = node->declare_parameter("my_number", params.my_number);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
""",
    )


@slides.slide(debug_boxes=False)
def details(slide):
    code_slide(
        slide,
        "ParameterDescriptor",
        "C++",
        """
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("minimal_param_node");
  auto params = Params{};

  auto param_desc  = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "Mine!";
  param_desc.additional_constraints  = "One of [world, base, home]";
  params.my_string = node->declare_parameter("my_string",
    params.my_string, param_desc);

  param_desc  = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc.description = "Who controls the universe?";
  param_desc.additional_constraints  = "A multiple of 23";
  params.my_number = node->declare_parameter("my_number",
    params.my_number, param_desc);
  //...
""",
    )


@slides.slide(debug_boxes=False)
def validate(slide):
    code_slide(
        slide,
        "Validation",
        "C++",
        """
auto const _ = node->add_on_set_parameters_callback(
  [](std::vector<rclcpp::Parameter> const& params)
  {
    for (auto const& param : params) {
      if(param.get_name() == "my_string") {
          auto const value = param.get_value<std::string>();
          auto const valid = std::vector<std::string>{"world", "base", "home"};
          if (std::find(valid.cbegin(), valid.cend(), value) == valid.end()) {
            auto result = rcl_interfaces::msg::SetParametersResult{};
            result.successful = false;
            result.reason = std::string("my_string: {")
              .append(value)
              .append("} not one of: [world, base, home]");
            return result;
          }
        }
      }
    return rcl_interfaces::msg::SetParametersResult{};
  });
""",
    )


@slides.slide(debug_boxes=False)
def validate(slide):
    content = logo_header_slide(slide, "")
    content.box().text("30 lines of C++ boilderpate per parameter")


@slides.slide(debug_boxes=False)
def part2(slide):
    section_title_slide(slide, "generate_\nparameter_library", "Part 2")


@slides.slide(debug_boxes=False)
def gpl(slide):
    code_slide(
        slide,
        "YAML",
        "",
        """
minimal_param_node:
    my_string: {
        type: string
        description: "Mine!"
        validation: {
            one_of<>: [["world", "base", "home"]]
        }
    }
    my_number: {
        type: int
        description: "Mine!"
        validation: {
            multiple_of_23: []
        }
    }
""",
    )


@slides.slide(debug_boxes=False)
def gpl(slide):
    code_slide(
        slide,
        "CMake Module",
        "cmake",
        """
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  minimal_param_node_parameters
  src/minimal_param_node.yaml
)

add_executable(minimal_node src/minimal_param_node.cpp)
target_link_libraries(minimal_node PRIVATE
  rclcpp::rclcpp
  minimal_param_node_parameters
)
""",
    )


@slides.slide(debug_boxes=False)
def gpl(slide):
    code_slide(
        slide,
        "C++ Usage",
        "C++",
        """
#include <rclcpp/rclcpp.hpp>
#include "minimal_param_node_parameters.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("minimal_param_node");
  auto param_listener =
    std::make_shared<minimal_param_node::ParamListener>(node);
  auto params = param_listener->get_params();

  // ...
""",
    )


@slides.slide(debug_boxes=False)
def part3(slide):
    section_title_slide(slide, "Boring?", "Part 3")


slides.render("parameters_talk.pdf")

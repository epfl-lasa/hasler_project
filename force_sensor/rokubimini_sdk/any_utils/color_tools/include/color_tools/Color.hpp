/*
 * Color.hpp
 *
 *  Created on: Dec 17, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

namespace color_tools
{
class Color
{
public:
  float r_ = 0.f;
  float g_ = 0.f;
  float b_ = 0.f;

public:
  Color()
  {
  }

  Color(float r, float g, float b) : r_(saturate(r)), g_(saturate(g)), b_(saturate(b))
  {
  }

  virtual ~Color()
  {
  }

  // Some predefined colors.
  static Color White()
  {
    return Color(1.f, 1.f, 1.f);
  }
  static Color Black()
  {
    return Color(0.f, 0.f, 0.f);
  }
  static Color Red()
  {
    return Color(1.f, 0.f, 0.f);
  }
  static Color Green()
  {
    return Color(0.f, 1.f, 0.f);
  }
  static Color Blue()
  {
    return Color(0.f, 0.f, 1.f);
  }
  static Color Yellow()
  {
    return Color(1.f, 1.f, 0.f);
  }
  static Color Pink()
  {
    return Color(1.f, 0.f, 1.f);
  }
  static Color Turquoise()
  {
    return Color(0.f, 1.f, 1.f);
  }

  /*!
   * Convert a value to a color. [0..1] is mapped to the rainbow colors [red..blue].
   * @param value value between 0 and 1.
   * @return equivalent color in the rainbow.
   */
  static Color FromValue(float value)
  {
    value = saturate(value);

    if (value < 0.25f)
    {
      const float x = 4.f * value;
      return Red() * (1.f - x) + Yellow() * x;
    }
    else if (value < 0.5f)
    {
      const float x = 4.f * (value - 0.25f);
      return Yellow() * (1.f - x) + Green() * x;
    }
    else if (value < 0.75f)
    {
      const float x = 4.f * (value - 0.5f);
      return Green() * (1.f - x) + Turquoise() * x;
    }
    else
    {
      const float x = 4.f * (value - 0.75f);
      return Turquoise() * (1.f - x) + Blue() * x;
    }
  }

  /*!
   * Mix this color with another one.
   * @param other other color.
   * @return resulting color.
   */
  Color mix(const Color& other) const
  {
    return Color(0.5f * (r_ + other.r_), 0.5f * (g_ + other.g_), 0.5f * (b_ + other.b_));
  }

  /*!
   * Multiply a color by a factor.
   * @param factor factor to multiply with.
   * @return resulting color.
   */
  Color operator*(float factor) const
  {
    return Color(factor * r_, factor * g_, factor * b_);
  }

  /*!
   * Add a color to this one.
   * @param other other color.
   * @return resulting color.
   */
  Color operator+(const Color& other) const
  {
    return Color(r_ + other.r_, g_ + other.g_, b_ + other.b_);
  }

protected:
  static float saturate(float value)
  {
    return std::max(0.f, std::min(1.f, value));
  }
};

}  // color_tools

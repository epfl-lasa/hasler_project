#pragma once

#include <stdarg.h> /* va_list, va_start, va_arg, va_end */
#include <stdio.h>  /* printf */
#include <string>   /* string */

namespace gtest_tools
{
enum class Color : unsigned int
{
  // Reset
  Color_Off = 0,  // Text Reset

  // Regular Colors
  Black,   // Black
  Red,     // Red
  Green,   // Green
  Yellow,  // Yellow
  Blue,    // Blue
  Purple,  // Purple
  Cyan,    // Cyan
  White,   // White

  // Bold
  BBlack,   // Black
  BRed,     // Red
  BGreen,   // Green
  BYellow,  // Yellow
  BBlue,    // Blue
  BPurple,  // Purple
  BCyan,    // Cyan
  BWhite,   // White

  // Underline
  UBlack,   // Black
  URed,     // Red
  UGreen,   // Green
  UYellow,  // Yellow
  UBlue,    // Blue
  UPurple,  // Purple
  UCyan,    // Cyan
  UWhite,   // White

  // Background
  On_Black,   // Black
  On_Red,     // Red
  On_Green,   // Green
  On_Yellow,  // Yellow
  On_Blue,    // Blue
  On_Purple,  // Purple
  On_Cyan,    // Cyan
  On_White,   // White

  // High Intensity
  IBlack,   // Black
  IRed,     // Red
  IGreen,   // Green
  IYellow,  // Yellow
  IBlue,    // Blue
  IPurple,  // Purple
  ICyan,    // Cyan
  IWhite,   // White

  // Bold High Intensity
  BIBlack,   // Black
  BIRed,     // Red
  BIGreen,   // Green
  BIYellow,  // Yellow
  BIBlue,    // Blue
  BIPurple,  // Purple
  BICyan,    // Cyan
  BIWhite,   // White

  // High Intensity backgrounds
  On_IBlack,   // Black
  On_IRed,     // Red
  On_IGreen,   // Green
  On_IYellow,  // Yellow
  On_IBlue,    // Blue
  On_IPurple,  // Purple
  On_ICyan,    // Cyan
  On_IWhite    // White
};

static inline const char* getFormatStringFromColor(Color color)
{
  switch (color)
  {
    case Color::Color_Off:
      return "\e[0m";
    case Color::Black:
      return "\e[0;30m";
    case Color::Red:
      return "\e[0;31m";
    case Color::Green:
      return "\e[0;32m";
    case Color::Yellow:
      return "\e[0;33m";
    case Color::Blue:
      return "\e[0;34m";
    case Color::Purple:
      return "\e[0;35m";
    case Color::Cyan:
      return "\e[0;36m";
    case Color::White:
      return "\e[0;37m";
    case Color::BBlack:
      return "\e[1;30m";
    case Color::BRed:
      return "\e[1;31m";
    case Color::BGreen:
      return "\e[1;32m";
    case Color::BYellow:
      return "\e[1;33m";
    case Color::BBlue:
      return "\e[1;34m";
    case Color::BPurple:
      return "\e[1;35m";
    case Color::BCyan:
      return "\e[1;36m";
    case Color::BWhite:
      return "\e[1;37m";
    case Color::UBlack:
      return "\e[4;30m";
    case Color::URed:
      return "\e[4;31m";
    case Color::UGreen:
      return "\e[4;32m";
    case Color::UYellow:
      return "\e[4;33m";
    case Color::UBlue:
      return "\e[4;34m";
    case Color::UPurple:
      return "\e[4;35m";
    case Color::UCyan:
      return "\e[4;36m";
    case Color::UWhite:
      return "\e[4;37m";
    case Color::On_Black:
      return "\e[40m";
    case Color::On_Red:
      return "\e[41m";
    case Color::On_Green:
      return "\e[42m";
    case Color::On_Yellow:
      return "\e[43m";
    case Color::On_Blue:
      return "\e[44m";
    case Color::On_Purple:
      return "\e[45m";
    case Color::On_Cyan:
      return "\e[46m";
    case Color::On_White:
      return "\e[47m";
    case Color::IBlack:
      return "\e[0;90m";
    case Color::IRed:
      return "\e[0;91m";
    case Color::IGreen:
      return "\e[0;92m";
    case Color::IYellow:
      return "\e[0;93m";
    case Color::IBlue:
      return "\e[0;94m";
    case Color::IPurple:
      return "\e[0;95m";
    case Color::ICyan:
      return "\e[0;96m";
    case Color::IWhite:
      return "\e[0;97m";
    case Color::BIBlack:
      return "\e[1;90m";
    case Color::BIRed:
      return "\e[1;91m";
    case Color::BIGreen:
      return "\e[1;92m";
    case Color::BIYellow:
      return "\e[1;93m";
    case Color::BIBlue:
      return "\e[1;94m";
    case Color::BIPurple:
      return "\e[1;95m";
    case Color::BICyan:
      return "\e[1;96m";
    case Color::BIWhite:
      return "\e[1;97m";
    case Color::On_IBlack:
      return "\e[0;100m";
    case Color::On_IRed:
      return "\e[0;101m";
    case Color::On_IGreen:
      return "\e[0;102m";
    case Color::On_IYellow:
      return "\e[0;103m";
    case Color::On_IBlue:
      return "\e[0;104m";
    case Color::On_IPurple:
      return "\e[0;105m";
    case Color::On_ICyan:
      return "\e[0;106m";
    case Color::On_IWhite:
      return "\e[0;107m";
  }
  return "";
}

static inline void colored_fprintf(FILE* stream, Color color, const char* fmt, va_list args)
{
  fprintf(stream, "%s", getFormatStringFromColor(color));
  vfprintf(stream, fmt, args);
  fprintf(stream, "%s", getFormatStringFromColor(Color::Color_Off));  // Resets the terminal to default.
}

static inline void colored_printf(Color color, const char* fmt, va_list args)
{
  colored_fprintf(stdout, color, fmt, args);
}

static inline void colored_fprintf(FILE* stream, Color color, const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  colored_fprintf(stream, color, fmt, args);
  va_end(args);
}

static inline void colored_printf(Color color, const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  colored_printf(color, fmt, args);
  va_end(args);
}

static inline void colored_brackets(FILE* stream, Color bracketColor, std::string bracketText)
{
  // If text too long use abbreviation
  if (bracketText.length() > 8)
  {
    bracketText.resize(7);
    bracketText += ".";
  }
  // Add bracket at front, spaces in middle and bracket at end
  bracketText = "[ " + bracketText;
  while (bracketText.length() != 11)
    bracketText += " ";
  bracketText += "] ";

  // Print bracket
  colored_fprintf(stream, bracketColor, bracketText.c_str());
}

static inline void colored_fprintf_brackets(FILE* stream, Color bracketColor, std::string bracketText, Color textColor,
                                            const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  colored_brackets(stream, bracketColor, bracketText);
  colored_fprintf(stream, textColor, fmt, args);
  va_end(args);
}

static inline void colored_printf_brackets(Color bracketColor, std::string bracketText, Color textColor,
                                           const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  colored_brackets(stdout, bracketColor, bracketText);
  colored_printf(textColor, fmt, args);
  va_end(args);
}

static inline void colored_fprintf_arrow_brackets(FILE* stream, Color color, const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  colored_brackets(stream, color, "->");
  colored_fprintf(stream, color, fmt, args);
  va_end(args);
}

static inline void colored_printf_arrow_brackets(Color color, const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);
  colored_brackets(stdout, color, "->");
  colored_printf(color, fmt, args);
  va_end(args);
}

}  // namespace gtest_tools

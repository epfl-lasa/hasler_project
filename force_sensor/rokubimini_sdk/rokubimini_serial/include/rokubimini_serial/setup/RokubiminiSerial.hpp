#pragma once

// std
#include <cstdint>

// rokubimini
#include <rokubimini/setup/Rokubimini.hpp>

namespace rokubimini
{
namespace serial
{
namespace setup
{
using Rokubimini = rokubimini::setup::Rokubimini;

/**
 * @class RokubiminiSerial
 *
 * @brief The Rokubimini Serial Setup class.
 *
 * Inherits from the Rokubimini Setup class. It's used for
 * parsing a setup file and creating a Rokubimini Serial Setup
 * from it, by parsing specific serial attributes from the file
 * (port, baudrate, etc.).
*/

class RokubiminiSerial : public Rokubimini
{
public:
  /**
   * @fn RokubiminiSerial()
   *
   * @brief Default constructor of a Rokubimini Serial Setup.
   *
  */
  RokubiminiSerial() = default;
  ~RokubiminiSerial() override = default;

  /**
   * @fn void fromFile(const yaml_tools::YamlNode &yamlNode, const std::string &setupFile)
   *
   * @brief Parses the @param setupFile and gets the serial-specific information (port, baud rate) from it.
   *
   * @param yamlNode The specific yaml node to parse for serial
   * specifics.
   * @param setupFile The setup file.
  */
  void fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile) override;

  /**
   * @var std::string serialPort_
   *
   * @brief The serial port name.
  */
  std::string serialPort_{ "/dev/ttyUSB0" };

  /**
   * @var uint32_t baudRate_
   *
   * @brief The baud rate of the serial communication.
   *
  */
  uint32_t baudRate_{ 0 };
};
}  // namespace setup
}  // namespace serial
}  // namespace rokubimini

/*
 * types.hpp
 *
 *  Created on:   Jul 9, 2017
 *  Author(s):    Vassilios Tsounis, Christian Gehring, Klajd Lika
 */

#pragma once

namespace rokubimini
{
namespace serial
{
using Wrench = kindr::WrenchD;
using WrenchRaw = kindr::Wrench6<int>;
using Matrix6D = Eigen::Matrix<double, 6, 6>;

}  // namespace serial
}  // namespace rokubimini

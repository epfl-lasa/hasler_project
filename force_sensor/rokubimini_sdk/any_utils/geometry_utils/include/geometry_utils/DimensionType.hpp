/**
 * @authors     Remo Diethelm
 * @affiliation ANYbotics
 * @brief       Definition of dimensions.
 */

#pragma once

#include <std_utils/containers/CompileTimeMap.hpp>

namespace geometry_utils
{
enum class TranslationDimensionType
{
  Txy,
  Txyz
};
enum class RotationDimensionType
{
  Ry,
  Rrpy
};
enum class DimensionType
{
  Txy,
  Txyz,
  Ry,
  Rrpy,
  TxyRy,
  TxyzRy,
  TxyzRrpy
};

namespace internal
{
template <TranslationDimensionType td, DimensionType d>
using kvTdd = std_utils::KeyValuePair<TranslationDimensionType, DimensionType, td, d>;

template <DimensionType d, TranslationDimensionType td>
using kvDtd = std_utils::KeyValuePair<DimensionType, TranslationDimensionType, d, td>;

template <RotationDimensionType rd, DimensionType d>
using kvRdd = std_utils::KeyValuePair<RotationDimensionType, DimensionType, rd, d>;

template <DimensionType d, RotationDimensionType rd>
using kvDrd = std_utils::KeyValuePair<DimensionType, RotationDimensionType, d, rd>;

}  // namespace internal

using mapTranslationDimensionToDimensionType =
    std_utils::CompileTimeMap<TranslationDimensionType, DimensionType,
                              internal::kvTdd<TranslationDimensionType::Txy, DimensionType::Txy>,
                              internal::kvTdd<TranslationDimensionType::Txyz, DimensionType::Txyz>>;

using reduceDimensionToTranslationDimension =
    std_utils::CompileTimeMap<DimensionType, TranslationDimensionType,
                              internal::kvDtd<DimensionType::Txy, TranslationDimensionType::Txy>,
                              internal::kvDtd<DimensionType::TxyRy, TranslationDimensionType::Txy>,
                              internal::kvDtd<DimensionType::Txyz, TranslationDimensionType::Txyz>,
                              internal::kvDtd<DimensionType::TxyzRy, TranslationDimensionType::Txyz>,
                              internal::kvDtd<DimensionType::TxyzRrpy, TranslationDimensionType::Txyz>>;

using mapRotationDimensionToDimensionType =
    std_utils::CompileTimeMap<RotationDimensionType, DimensionType,
                              internal::kvRdd<RotationDimensionType::Ry, DimensionType::Ry>,
                              internal::kvRdd<RotationDimensionType::Rrpy, DimensionType::Rrpy>>;

}  // namespace geometry_utils

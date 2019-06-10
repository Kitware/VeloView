#include "vtkVelodyneAdvancedPacketInterpreter.h"

#include <vtkDoubleArray.h>
#include <vtkObjectFactory.h>
#include <vtkPointData.h>
#include <vtkPoints.h>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include "vtkDataPacket.h"
using namespace DataPacketFixedLength;

#include <boost/endian/arithmetic.hpp>
#include <boost/predef/detail/endian_compat.h>
#include <boost/preprocessor.hpp>
#include <type_traits>

#include <cstring>

#include <iostream>

// include only to go the structure VelodyneSpecificFrameInformation before merging code
#include <vtkVelodynePacketInterpreter.h>

#define DEBUGMSG(msg) std::cout << msg << " [" << __LINE__ << "]\n";

// clang-format off
//------------------------------------------------------------------------------
// General macros constants.
//------------------------------------------------------------------------------
//! @brief Lengths in the headers are given in units of 32-bit words.
#define VAPI_BYTES_PER_HEADER_WORD 4u

//! @brief Get the first set bit of an integral value.
#define FIRST_SET_BIT(n) (n - (n & (n-1)))

//! @brief Memory step size array memory allocation.
#define MEM_STEP_SIZE 0x40000u

//------------------------------------------------------------------------------
// Type Conversion Structs
//------------------------------------------------------------------------------
// Private members are stored using boost endian arithmetic types. To make these
// transparent, getters return native values. Create structs to handle the
// conversions in the getters.
//
// These are currently unused due to non-compliance with C++11 on our Windows
// matchine. When Windows is updated, the VAPI_GET_NATIVE_UINT macro below can
// be removed and the VAPI_GET_RAW macro's return type can be updated to
// "AsNative<decltype(attr)>::type".
template <typename T>
struct AsNative
{
  typedef T type;
};


#define VAPI_DECLARE_BIG_TO_NATIVE(typ)       \
  template <>                                 \
  struct AsNative<boost::endian::big_ ## typ> \
  {                                           \
    typedef typ type;                         \
  };

VAPI_DECLARE_BIG_TO_NATIVE(uint16_t)
VAPI_DECLARE_BIG_TO_NATIVE(uint32_t)
VAPI_DECLARE_BIG_TO_NATIVE(uint64_t)

//------------------------------------------------------------------------------
// Accessor macros.
//------------------------------------------------------------------------------
//! @brief Simple getter for uninterpretted values that returns native types.
// #define VAPI_GET_RAW(attr) typename AsNative<decltype(attr)>::type Get ## attr () const { return this->attr; }
#define VAPI_GET_RAW(attr) decltype(attr) Get ## attr () const { return this->attr; }

//! @brief Get native unsigned integers.
// TODO
// Remove this once we can use AsNative in the return type of VAPI_GET_RAW. See
// notes above about using AsNative.
#define VAPI_GET_NATIVE_UINT(n, attr) uint ## n ##_t Get ## attr () const { return this->attr; }

//! @brief Get a const reference.
#define VAPI_GET_CONST_REF(attr) decltype(attr) const & Get ## attr () const { return this->attr; }

//! @brief Getter for enum values that are stored as raw values internally.
#define VAPI_GET_ENUM(enumtype, attr) enumtype Get ## attr () const { return to ## enumtype (static_cast<uint8_t>(this->attr)); }

//! @brief Get a header length in bytes.
#define VAPI_GET_LENGTH(attr) size_t Get ## attr () const { return (this->attr * VAPI_BYTES_PER_HEADER_WORD); }



//------------------------------------------------------------------------------
// Enum macros.
//------------------------------------------------------------------------------
/*
 * Define some macros to facilitate maintenance of different enum types. These
 * allow a single macro to define an enum type with values, an overloaded
 * ToString function to convert macro values to strings, and a templated
 * To<enum> function to convert integral values to enums.
 *
 * For details of the Boost preprocessing macros, see
 * https://www.boost.org/doc/libs/1_67_0/libs/preprocessor/doc/AppendixA-AnIntroductiontoPreprocessorMetaprogramming.html
 */

//! @brief Internal macro for defining enum values.
#define DEFINE_ENUM_VALUES_INTERNAL(r, prefix, pair) \
  BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair)) = BOOST_PP_TUPLE_ELEM(1, pair)

//! @brief Macro for defining enum values.
#define DEFINE_ENUM_VALUES(name, prefix, enumerators) \
  enum name {                                         \
    BOOST_PP_SEQ_ENUM(                                \
      BOOST_PP_SEQ_TRANSFORM(                         \
        DEFINE_ENUM_VALUES_INTERNAL,                  \
        prefix,                                       \
        enumerators                                   \
      )                                               \
    )                                                 \
  };


//! @brief Internal macro for defining enum string conversions.
#define DEFINE_ENUM_STRING_CASES_INTERNAL(r, prefix, pair) \
  case BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair)): return BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(0, pair));


//! @brief Macro for defining enum string conversions.
#define DEFINE_ENUM_TO_STRING(name, prefix, enumerators)                                \
  inline                                                                                \
  char const * toString(name const x)                                                   \
  {                                                                                     \
    switch(x)                                                                           \
    {                                                                                   \
      BOOST_PP_SEQ_FOR_EACH(                                                            \
        DEFINE_ENUM_STRING_CASES_INTERNAL,                                              \
        prefix,                                                                         \
        enumerators                                                                     \
      )                                                                                 \
      default: return "<unrecognized enum value of type " BOOST_PP_STRINGIZE(name) ">"; \
    }                                                                                   \
  }


//! @brief Internal macro for converting integral values to enums.
#define DEFINE_VALUE_TO_ENUM_CASES_INTERNAL(r, prefix, pair) \
  case static_cast<T>(BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair))): return BOOST_PP_CAT(prefix, BOOST_PP_TUPLE_ELEM(0, pair));


//! @brief Macro for converting integral values to enums.
#define DEFINE_VALUE_TO_ENUM(name, prefix, enumerators, default_value) \
  template <typename T>                                                \
  inline                                                               \
  name to ## name(T const x)                                           \
  {                                                                    \
    switch(x)                                                          \
    {                                                                  \
      BOOST_PP_SEQ_FOR_EACH(                                           \
        DEFINE_VALUE_TO_ENUM_CASES_INTERNAL,                           \
        prefix,                                                        \
        enumerators                                                    \
      )                                                                \
      default: return BOOST_PP_CAT(prefix, default_value);             \
    }                                                                  \
  }

/*!
 * @brief Define enum type with string and value conversion functions.
 * @param name        The typename of the enum. This will also be concatenated
 *                    with "To" to define a function that converts integral
 *                    values to this enum type. For example. the name "Foo" will
 *                    define "Foo toFoo(T x)".
 * @param prefix      The prefix to attach to all values of the enum. This may
 *                    be an empty string.
 * @param enumerators A sequence of name-value pairs in double-parentheses, e.g.
 *                    ((a, 1))((b, 2))((c , 4)). All valid enum values may be
 *                    used, e.g. ((a, (1<<0)))((b, (1<<1))), etc.
 */
#define DEFINE_ENUM(name, prefix, enumerators, default_value)    \
  DEFINE_ENUM_VALUES(name, prefix, enumerators)                  \
  DEFINE_ENUM_TO_STRING(name, prefix, enumerators)               \
  DEFINE_VALUE_TO_ENUM(name, prefix, enumerators, default_value)

//------------------------------------------------------------------------------
// Info array macros.
//------------------------------------------------------------------------------
//! @ brief List of field macros for using in other macros.
#define VAPI_INFO_ARRAYS \
	(Xs)                    \
	(Ys)                    \
	(Zs)                    \
	(Distances)             \
	(Azimuths)              \
	(VerticalAngles)        \
	(Confidences)           \
	(Intensities)           \
	(Reflectivities)        \
	(DistanceTypes)         \
	(ChannelNumbers)        \
	(Noises)                \
	(Powers)                \
	(Pseqs)                 \
	(TimeFractionOffsets)

//! @brief Wrapper around BOOST_PP_CAT for use with BOOST_PP_SEQ_TRANSFORM.
#define VAPI_ADD_PREFIX(index, prefix, elem) BOOST_PP_CAT(prefix, elem)

//! @brief The field array names prefixed with "this->INFO_".
#define VAPI_INFO_ARRAY_MEMBERS \
	BOOST_PP_SEQ_TRANSFORM(VAPI_ADD_PREFIX, this->INFO_, VAPI_INFO_ARRAYS)

/*!
 * @brief     Expand a macro with the given data for each field array.
 * @param[in] action The macro to expand. It must accept 3 arguments. The first
 *                   is the index of the array in VAPI_INFO_ARRAYS, the second
 *                   is the data and the third is the array name.
 * @param[in] data   Any data to pass through as the second argument to
 *                   "action".
 */
#define VAPI_FOREACH_INFO_ARRAY(action, data) \
    BOOST_PP_SEQ_FOR_EACH(action, data, VAPI_INFO_ARRAY_MEMBERS)

//------------------------------------------------------------------------------
// Global constants.
//------------------------------------------------------------------------------
//! @brief Lookup table for the number of set bits in a byte.
static uint8_t const SET_BITS_IN_BYTE[0x100] = {
  0, 1, 1, 2,   1, 2, 2, 3,   1, 2, 2, 3,   2, 3, 3, 4,   1, 2, 2, 3,   2, 3, 3, 4,   2, 3, 3, 4,   3, 4, 4, 5,
  1, 2, 2, 3,   2, 3, 3, 4,   2, 3, 3, 4,   3, 4, 4, 5,   2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,
  1, 2, 2, 3,   2, 3, 3, 4,   2, 3, 3, 4,   3, 4, 4, 5,   2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,
  2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,   3, 4, 4, 5,   4, 5, 5, 6,   4, 5, 5, 6,   5, 6, 6, 7,
  1, 2, 2, 3,   2, 3, 3, 4,   2, 3, 3, 4,   3, 4, 4, 5,   2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,
  2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,   3, 4, 4, 5,   4, 5, 5, 6,   4, 5, 5, 6,   5, 6, 6, 7,
  2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,   3, 4, 4, 5,   4, 5, 5, 6,   4, 5, 5, 6,   5, 6, 6, 7,
  3, 4, 4, 5,   4, 5, 5, 6,   4, 5, 5, 6,   5, 6, 6, 7,   4, 5, 5, 6,   5, 6, 6, 7,   5, 6, 6, 7,   6, 7, 7, 8
};

//------------------------------------------------------------------------------
//! @brief The number of bits in a byte.
static uint8_t const BITS_PER_BYTE = sizeof(SET_BITS_IN_BYTE[0]);

//------------------------------------------------------------------------------
// Typedefs and enums.
//
// Use explicit enumerations to avoid all possible confusion that might arise
// with simple numerical tests. The compiler should optimize the checks.
//
// Use the abbreviations given in the specification as prefixes.
//------------------------------------------------------------------------------

/*!
 * @brief Model identification code.
 * @todo  The VELARRAY value is not part of the specification but it was used in
 *        provided pcap files.
 */
DEFINE_ENUM(
  ModelIdentificationCode,
  MIC_,
  ((RESERVED , 0))
  ((VLP16    , 1))
  ((VLP16_HD , 2))
  ((VLP32A   , 3))
  ((VLP32B   , 4))
  ((VELARRAY , 0x31)),
  RESERVED
)

//------------------------------------------------------------------------------
//! @brief Horizontal direction of Lidar.
DEFINE_ENUM(
  HorizontalDirection,
  HDIR_,
  ((CLOCKWISE         , 0))
  ((COUNTER_CLOCKWISE , 1)),
  CLOCKWISE
)

//------------------------------------------------------------------------------
//! @brief Vertical direction of Lidar.
DEFINE_ENUM(
  VerticalDirection,
  VDIR_,
  ((UP   , 0))
  ((DOWN , 1)),
  UP
)

//------------------------------------------------------------------------------
//! @brief Firing mode.
DEFINE_ENUM(
  FiringMode,
  FM_,
  ((PASSIVE  , 0))
  ((NORMAL   , 1))
  ((RESERVED , 2))
  ((INDEX    , 15)),
  RESERVED
)

//------------------------------------------------------------------------------
//! @brief Channel status flags.
DEFINE_ENUM(
  ChannelStatus,
  STAT_,
  ((OBSTRUCTION_DETECTED , 0))
  ((FAULT_DETECTED       , 1))
  ((POWER_NOT_PERFECT    , 2))
  ((RESERVED             , 3)),
  RESERVED
)

//------------------------------------------------------------------------------
//! @brief Mask format to specify number and type of values in intensity set.
// Bits 3-15 are currently reserved.
DEFINE_ENUM(
  IntensityType,
  ISET_,
  ((REFLECTIVITY , (0u)))
  ((INTENSITY    , (1u << 0)))
  ((CONFIDENCE   , (1u << 1)))
  ((RESERVED     , (1u << 2))),
  RESERVED
  )

//------------------------------------------------------------------------------
//! @brief Mask format to specify values in returned distance set.
// Bists 4-5 are currently reserved.
DEFINE_ENUM(
  DistanceType,
  DSET_,
  ((FIRST            , (0u)))
  ((STRONGEST        , (1u << 0)))
  ((SECOND_STRONGEST , (1u << 1)))
  ((LAST             , (1u << 2)))
  ((RESERVED         , (1u << 3))),
  RESERVED
)

// clang-format on

//------------------------------------------------------------------------------
// Convenience functions.
//------------------------------------------------------------------------------
/*!
 * @brief     Convert degrees to radians.
 * @param[in] degrees The input value in degrees.
 * @return    The input value converted to radians.
 */
inline double
degreesToRadians(double degrees)
{
  return (degrees * vtkMath::Pi()) / 180.0;
}

//------------------------------------------------------------------------------
/*!
 * @brief     Get the index of a bit in a bitmask.
 * @tparam    T    The input and return types.
 * @param[in] mask The mask of set bits.
 * @param[in] bit  A value with a single set bit the index of which to
 *                 determine.
 *
 * The distances and intensities included in the packet are indicated by
 * bitmasks. Included values are ordered by their corresponding bit in the mask.
 * The index of a value in a list is therefore equal to the number of set
 * preceding bits.
 */
template <typename T, typename S>
T
indexOfBit(T value, S bit)
{
  T castBit       = static_cast<T>(bit);
  T precedingBits = (castBit - 1) & value;
  // Count the bits set before the target bit. This will be the index of the set
  // bit.
  if (sizeof(T) > BITS_PER_BYTE)
  {
    T byteMask = (1 << BITS_PER_BYTE) - 1;
    T index    = 0;
    while (precedingBits)
    {
      index += SET_BITS_IN_BYTE[(precedingBits & byteMask)];
      precedingBits >>= BITS_PER_BYTE;
    }
    return index;
  }
  else
  {
    return SET_BITS_IN_BYTE[precedingBits];
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief     Return the first set bit in a value.
 * @param[in] x The value.
 * @return    A value with the first set bit.
 */
template <typename T>
inline T
firstSetBit(T x)
{
  return (x - (x & (x - 1)));
}

//------------------------------------------------------------------------------
/*!
 * @brief     Set a target value from a range of bits in another value.
 * @tparam    TS     The source value type.
 * @tparam    TD     The destination value type.
 * @param[in] source The source value from which to set the destination value.
 * @param[in] offset The offset of the least significant bit.
 * @param[in] number The number of bits to set.
 * @return    The value.
 */
template <typename T>
inline T
bitRangeValue(T const & source, uint8_t offset, uint8_t number)
{
  T const mask = (static_cast<T>(1) << number) - 1;
  return (source >> offset) & mask;
}

//------------------------------------------------------------------------------
/*!
 * @brief     Safely perform a reinterpret cast on a header.
 * @tparam    T          The header type.
 * @param[in] data       A pointer to the input data to cast.
 * @param[in] dataLength The length of the data.
 * @param[in] index      The index of the start of the data.
 * @return    Returns a null pointer if any validity check fails.
 *
 * This checks that the data meets the minimum required length before
 * reinterpret_cast'ing the data to avoid segfaults. After the cast, the
 * internal consistency of the casted object is also checked.
 */
template <typename T>
inline T const *
reinterpretCastWithChecks(uint8_t const * data, size_t dataLength, size_t index)
{
  if (index > dataLength)
  {
    return nullptr;
  }
  data += index;
  dataLength -= index;
  // Check that there are enough bytes to safely interpret the header.
  if (dataLength < T::MinimumRequiredLength())
  {
    return nullptr;
  }
  T const * objectPtr = reinterpret_cast<T const *>(data);
  if (!objectPtr->IsValid())
  {
    return nullptr;
  }
  return objectPtr;
}

//------------------------------------------------------------------------------
/*!
 * @brief Advance the index by a header's HLEN if there is enough data,
 *        otherwise return.
 */
#define ADVANCE_INDEX_BY_HLEN_OR_RETURN(                                       \
  dataLength, index, objectPtr, returnValue)                                   \
  auto hlen = objectPtr->GetHlen();                                            \
  if (hlen > (dataLength - index))                                             \
  {                                                                            \
    return returnValue;                                                        \
  }                                                                            \
  else                                                                         \
  {                                                                            \
    index += hlen;                                                             \
  }

//------------------------------------------------------------------------------
// PayloadHeader
//------------------------------------------------------------------------------
/*!
 * @brief Payload header of the VLP Advanced data packet format.
 *
 * All lengths count 32-bit words, e.g. a Glen value of 4 indicates that the
 * firing group header consists of 4 32-bit words. The raw values are converted
 * to counts with the VAPI_BYTES_PER_HEADER_WORD constant.
 */
#pragma pack(push, 1)
class PayloadHeader
{
private:
  //@{
  /*!
   * @brief Private members.
   *
   * Ver : Protocol version. Hlen : Header length (min: 5) Nxdhr : Next header
   * type. Glen : Firing group header length. Flen : Firing header length. Mic :
   * Model Identification Code Tstat : Time status (TBD). Dset : Distance set.
   * Iset : Intensity set. Tref : Time reference. Pset : Payload sequence
   * number.
   */
  uint8_t VerHlen;
  uint8_t Nxhdr;
  uint8_t GlenFlen;
  uint8_t Mic;
  uint8_t Tstat;
  uint8_t Dset;
  boost::endian::big_uint16_t Iset;
  boost::endian::big_uint64_t Tref;
  boost::endian::big_uint32_t Pseq;
  //@}

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * HLEN, GLEN and FLEN are returned in bytes, not the number of 32-bit words.
   */
  uint8_t
  GetVer() const
  {
    return bitRangeValue<uint8_t>(this->VerHlen, 4, 4);
  }
  uint8_t
  GetHlen() const
  {
    return bitRangeValue<uint8_t>(this->VerHlen, 0, 4) *
           VAPI_BYTES_PER_HEADER_WORD;
  }
  VAPI_GET_RAW(Nxhdr)
  uint8_t
  GetGlen() const
  {
    return bitRangeValue<uint8_t>(this->GlenFlen, 4, 4) *
           VAPI_BYTES_PER_HEADER_WORD;
  }
  uint8_t
  GetFlen() const
  {
    return bitRangeValue<uint8_t>(this->GlenFlen, 0, 4) *
           VAPI_BYTES_PER_HEADER_WORD;
  }
  VAPI_GET_ENUM(ModelIdentificationCode, Mic);
  VAPI_GET_RAW(Tstat)
  VAPI_GET_RAW(Dset)
  VAPI_GET_NATIVE_UINT(16, Iset)
  VAPI_GET_NATIVE_UINT(64, Tref)
  VAPI_GET_NATIVE_UINT(32, Pseq)
  //@}

  //! @brief Get the DSET mask (or the count if DSET is not a mask).
  uint8_t
  GetDsetMask() const
  {
    return bitRangeValue<uint8_t>(this->Dset, 0, 6);
  }

  //! @brief The number of bytes per value in a return.
  uint8_t
  GetDistanceSizeInBytes() const
  {
    return (bitRangeValue<uint8_t>(this->Dset, 9, 1) ? 3 : 2);
  }

  //! @brief True if the distance set includes a mask, false if a count.
  bool
  IsDsetMask() const
  {
    return (!bitRangeValue<uint8_t>(this->Dset, 6, 1));
  }

  //! @brief Get the number of distances in each firing group.
  uint8_t
  GetDistanceCount() const
  {
    // The "mask" is actually a count if IsDsetMask is false.
    auto mask = this->GetDsetMask();
    return (this->IsDsetMask()) ? SET_BITS_IN_BYTE[mask] : mask;
  }

  //! @brief Get the number of intensities in each firing.
  uint8_t
  GetIntensityCount() const
  {
    if (this->GetDset() == 0)
    {
      return 0;
    }
    else
    {
      return SET_BITS_IN_BYTE[this->Iset];
    }
  }

  //! @brief The the number of bytes per firing return.
  size_t
  GetNumberOfBytesPerFiringReturn() const
  {
    size_t bytesPerDistance = this->GetDistanceSizeInBytes();
    size_t icount           = this->GetIntensityCount();
    return bytesPerDistance + icount;
  }

  //! @brief The number of bytes of data per firing.
  size_t
  GetNumberOfDataBytesPerFiring() const
  {
    size_t dcount         = this->GetDistanceCount();
    size_t bytesPerReturn = this->GetNumberOfBytesPerFiringReturn();
    return dcount * bytesPerReturn;
  }

  //! @brief The total number of bytes per firing (header + data).
  size_t
  GetNumberOfBytesPerFiring() const
  {
    return this->GetNumberOfDataBytesPerFiring() + this->GetFlen();
  }

  /*!
   * @brief Returns true if the packet header appears to be a valid version 1
   *        header.
   */
  inline bool
  IsValid() const
  {
    return (this->GetVer() == 1) && (this->GetHlen() >= 5) &&
           (this->GetGlen() >= 2);
  }

  //! @brief Get the minimum required length of this header, in bytes.
  static size_t
  MinimumRequiredLength()
  {
    return 20;
  }

  //! @brief An upper bound for the maximum number of data points in a packet.
  static size_t
  MaximumNumberOfPointsPerPacket()
  {
    // Max packet size divided by minimum distance size. The true max is less
    // than this due to packet and payload headers and other data, but this is
    // just to get an upper bound. An exact value will not avoid the final
    // resize before pushing the PolyData.
    return 0x10000 / 2;
  }
};
#pragma pack(pop)

//------------------------------------------------------------------------------
// ExtensionHeader
//------------------------------------------------------------------------------
/*!
 * @brief Extension header of the VLP Advanced data packet format.
 *
 * This should be subclassed to handle different types of extensions as
 * specified by the NXHDR field of the payload header.
 */
#pragma pack(push, 1)
class ExtensionHeader
{
private:
  //! @brief The length of the extension header.
  uint8_t Hlen;

  //! @brief Nxhdr The next header type (same as PayloadHeader).
  uint8_t Nxhdr;

  /*!
   * @brief Extension header data value.
   *
   * The data field must end on a 32-bit boundary. The format of the data is
   * extension-specific and determined by the NXHDR value of the previous header
   * (either the payload header or a preceding extension header).
   */
  uint8_t const * Data;

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * HLEN is returned in bytes, not the number of 32-bit words.
   */
  VAPI_GET_LENGTH(Hlen)
  VAPI_GET_RAW(Nxhdr)
  VAPI_GET_RAW(Data)
  //@}

  //! @brief Get the minimum required length of this header, in bytes.
  static size_t
  MinimumRequiredLength()
  {
    return 2;
  }

  //! @brief Check if this extension header appears to be valid.
  inline bool
  IsValid() const
  {
    // Empty extension headers make no sense so consider them invalid.
    return (this->GetHlen() > 2);
  }
};
#pragma pack(pop)

//------------------------------------------------------------------------------
// FiringGroupHeader
//------------------------------------------------------------------------------
/*!
 * @brief Firing group header of the VLP Advanced data packet format.
 */
#pragma pack(push, 1)
class FiringGroupHeader
{
private:
  //@{
  /*!
   * @brief Private members.
   *
   * Toffs: Unsigned time fraction offset from payload timestamp to firing time
   * of the Firing Group in units of 64 ns.
   *
   * FcntFspn: The first five most significant bits are FCNT, where (FCNT+1) is
   * the number of firings in the firing group.
   *
   * The first three least significant bits are FSPN, where (FSPN + 1) is the
   * count (span) of co-channels fired simultaneously in the Firing Group.
   *
   * Fdly: If FDLY is zero, all channels in the Firing Group were fired
   * simultaneously and the FSPN value may be ignored as there is no need to
   * calculate a per-channel time offset.
   *
   * If FDLY is non-zero, the channels were fired in co-channel groups separated
   * by FDLY. The span of each co-channel group is (FSPN+1). For a rolling
   * firing where each channel is fired separately, FSPN is 0. For a rolling
   * firing where two channels are fired together FSPN is 1. For a rolling
   * fireing where eight channels are fired together FSPN is 7.
   *
   * Hdir: Horizontal direction , 0: Clockwise, 1: Counter-clockwise (1 bit)
   *
   * Vdir: Vertical direction , 0: Upward, 1: Downward (1 bit)
   *
   * Vdfl: Vertical deflection angle (0.01 degree increments) [0..16383]
   *
   * Azm: Azimuth (0.01 degree increments) [0..35999]
   */
  boost::endian::big_uint16_t Toffs;
  uint8_t FcntFspn;
  uint8_t Fdly;
  boost::endian::big_uint16_t HdirVdirVdfl;
  boost::endian::big_uint16_t Azm;

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * - TOFFS is returned in nanoseconds. - HDIR and VDIR are returned as their
   * respective enums. - VDFL and AZM are returned as an integral number of
   * hundredths of degrees.
   */

  // Cast to 32-bit required to hold all values.
  uint32_t
  GetToffs() const
  {
    return static_cast<uint32_t>(this->Toffs) * 64u;
  }
  uint8_t
  GetFcnt() const
  {
    return bitRangeValue<uint8_t>(this->FcntFspn, 3, 5) + 1;
  }
  uint8_t
  GetFspn() const
  {
    return bitRangeValue<uint8_t>(this->FcntFspn, 0, 3) + 1;
  }
  VAPI_GET_RAW(Fdly)
  HorizontalDirection
  GetHdir() const
  {
    return toHorizontalDirection(
      bitRangeValue<uint16_t>(this->HdirVdirVdfl, 15, 1));
  }
  VerticalDirection
  GetVdir() const
  {
    return toVerticalDirection(
      bitRangeValue<uint16_t>(this->HdirVdirVdfl, 14, 1));
  }
  uint16_t
  GetVdfl() const
  {
    return bitRangeValue<uint16_t>(this->HdirVdirVdfl, 0, 14);
  }
  VAPI_GET_NATIVE_UINT(16, Azm)
  //@}

  //@{
  //! @brief Convenience accessors to get angles in degrees.
  double
  GetVerticalDeflection() const
  {
    return this->GetVdfl() * 0.01;
  }
  double
  GetAzimuth() const
  {
    return this->Azm * 0.01;
  }
  //@}

  //! @brief Get the minimum required length of this header, in bytes.
  static size_t
  MinimumRequiredLength()
  {
    return 8;
  }

  //! @brief True if the firing group appears to be valid.
  inline bool
  IsValid() const
  {
    return (this->GetFcnt() > 0);
  }
};
#pragma pack(pop)

//------------------------------------------------------------------------------
// FiringHeader
//------------------------------------------------------------------------------
/*!
 * @brief Firing header of the VLP Advanced data packet format.
 */
#pragma pack(push, 1)
class FiringHeader
{
private:
  //@{
  /*!
   * @brief Private members.
   *
   * Lcn : Logical channel number. Fm : Firing mode. Pwr : Power level. Nf :
   * Noise factor. Stat : Channel status flag.
   */
  //@}

  uint8_t Lcn;
  uint8_t FmPwr;
  uint8_t Nf;
  uint8_t Stat;

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * FM and STAT are returned as their respective enums.
   */
  VAPI_GET_RAW(Lcn)
  FiringMode
  GetFm() const
  {
    return toFiringMode(bitRangeValue<uint8_t>(this->FmPwr, 4, 4));
  }
  uint8_t
  GetPwr() const
  {
    return bitRangeValue<uint8_t>(this->FmPwr, 0, 4);
  }
  VAPI_GET_RAW(Nf)
  VAPI_GET_ENUM(ChannelStatus, Stat)
  //@}

  //! @brief Get the minimum required length of this header, in bytes.
  static size_t
  MinimumRequiredLength()
  {
    return 4;
  }

  //! @brief True if the firing head appears to be valid.
  inline bool
  IsValid() const
  {
    return true;
  }
};
#pragma pack(pop)

//------------------------------------------------------------------------------
// FiringReturn
//------------------------------------------------------------------------------
/*!
 * @brief Firing return of the VLP Advanced data packet format.
 * @todo  Consider using polymorphic types to avoid wasted space.
 *
 * The values in this struct may be encoded in the packet with 16 or 24 bits.
 * The size is determined at runtime by parsing the payload headers so a
 * template cannot be used here. The current implementation uses types large
 * enough to hold all possible values.
 */
#pragma pack(push, 1)
class FiringReturn
{
private:
  //! @brief The packed data with the return values.
  uint8_t const * Data;

public:
  //@{
  //! @brief Getters.
  VAPI_GET_CONST_REF(Data);
  //@}

  /*!
   * @brief Get the distance from this firing.
   *
   * The distance is encoded in a variable number of bytes (2-3). To avoid
   * dealing with conditional template parameters, we parse the value byte-by-
   * byte.
   *
   * The Advanced Packet Format specification states that all multibyte values
   * are in Big Endian (network) byte order.
   */
  template <typename T>
  T
  GetDistance(uint8_t bytesPerDistance) const
  {
    T distance = this->Data[0];
    for (size_t i = 1; i < bytesPerDistance; ++i)
    {
      distance = (distance * 0x100) + this->Data[i];
    }
    return distance;
  }

  // Default return type is 32-bit because the type may be either 16 or 24 bit.

  //! @brief Get an intensity from this firing by index.
  template <typename T>
  T
  GetIntensity(uint8_t bytesPerDistance, uint8_t i) const
  {
    return this->Data[bytesPerDistance + i];
  }

  //! @brief Get an intensity from this firing by type.
  template <typename T>
  T
  GetIntensity(uint8_t bytesPerDistance, uint8_t iset, IntensityType type) const
  {
    // Check that the value is actually present otherwise the calculated index
    // will yield a different value or end up out of range.
    if (!(iset & type))
    {
      // TODO: properly handle this exception
      return 0;
      // throw std::out_of_range("requested intensity type is not in the
      // intensity set");
    }
    uint8_t i = 0;

    // Count the number of bits set before the requested one. `mask` constains a
    // single set bit. By decrementing the value, we obtain a mask over all
    // preceding bits which we can then count to get the offset of the requested
    // intensity.
    uint8_t mask = static_cast<uint8_t>(type);
    if (mask)
    {
      mask--;
      i += SET_BITS_IN_BYTE[iset & mask];
    }
    return this->Data[bytesPerDistance + i];
  }

  FiringReturn(uint8_t const * data) : Data(data) {};
};
#pragma pack(pop)

//------------------------------------------------------------------------------
// FrameTracker
//------------------------------------------------------------------------------
/*!
 * @brief Object to track information related to frame changes. All frame change
 *        logic should be kept here.
 *
 * The current logic was taken from the VelArray branch.
 */
class FrameTracker
{
private:
  bool HasLastState;
  HorizontalDirection LastHorDir;
  VerticalDirection LastVertDir;
  int LastSlope;
  decltype(std::declval<FiringGroupHeader>().GetAzm()) LastAzimuth;
  decltype(std::declval<FiringGroupHeader>().GetVdfl()) LastVertDeflection;

public:
  FrameTracker()
  {
    this->Reset();
  }

  //! @brief Reset frame detection.
  void
  Reset()
  {
    this->HasLastState = false;
    this->LastSlope      = 0;
    this->LastAzimuth    = static_cast<decltype(this->LastAzimuth)>(-1);
  }

  /*!
   * @brief     Update the frame tracker.
   * @param[in] payloadHeader     The payload header, used to determine sensor
   *                              model.
   * @param[in] firingGroupHeader The firing group header to inspect for frame
   *                              changes.
   * @return    True if a new frame is detected, false otherwise.
   */
  bool
  Update(
    PayloadHeader const * const payloadHeader,
    FiringGroupHeader const * const firingGroupHeader)
  {
    // Get and update all member values here to avoid doing so in various blocks
    // below just before returning.

    decltype(this->LastAzimuth) azimuth     = firingGroupHeader->GetAzm(),
                                lastAzimuth = this->LastAzimuth;

    decltype(this->LastVertDeflection) vertDeflection = firingGroupHeader->GetVdfl(),
                                       lastVertDeflection = this->LastVertDeflection;
    // Follow the logic of the legacy interpreter.
    int slope = static_cast<int>(azimuth) - static_cast<int>(lastAzimuth),
        lastSlope = this->LastSlope;

    HorizontalDirection horDir = firingGroupHeader->GetHdir(),
                        lastHorDir = this->LastHorDir;

    VerticalDirection vertDir = firingGroupHeader->GetVdir(),
                      lastVertDir = this->LastVertDir;

    bool hasLastState = this->HasLastState;

    this->LastAzimuth = azimuth; // static_cast<decltype(this->LastAzimuth)>(-1);
    this->LastVertDeflection = vertDeflection;
    // this->LastSlope   = slope;
    this->LastHorDir = horDir;
    this->LastVertDir = vertDir;
    this->HasLastState = true;

    if (! hasLastState)
    {
      return false;
    }


    // VelArray
    ModelIdentificationCode mic = payloadHeader->GetMic();
    if (mic == ModelIdentificationCode::MIC_VELARRAY || firingGroupHeader->GetVdfl() != 0)
    {
      // The frame split if either the vertical direction changes (vertical
      // scanning) OR the horizontal direction changes without a change in the
      // vertical deflection (pure horizontal scanning).
      return (
          (vertDir != lastVertDir) ||
          (horDir != lastHorDir && vertDeflection == lastVertDeflection)
        );
    }

    // Not VelArray
    else
    {
      // // New frame when the azimuth rolls over.
      // if (azimuth == 0 and lastAzimuth != 0)
      // {
      //   return true;
      // }
      // else
      // {
      //   return this->HasLastHorDir ? (horDir != lastHorDir) : false;
      // }

      // Old logic from VelArray FramingState. This doesn't seem to work for
      // other sensors.
      if (slope == 0)
      {
        return false;
      }

      int isSameSlopeDir = slope * lastSlope;
      if (isSameSlopeDir > 0)
      {
        return false;
      }

      else if (isSameSlopeDir < 0)
      {
        this->LastSlope = 0;
        return true;
      }

      if (this->LastSlope == 0 && slope != 0)
      {
        this->LastSlope = slope;
        return false;
      }
      vtkGenericWarningMacro("Unhandled sequence in framing state.");
      return false;
    }

    return false;
  }
};

//------------------------------------------------------------------------------
// vtkVelodyneAdvancedPacketInterpreter methods.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
vtkStandardNewMacro(vtkVelodyneAdvancedPacketInterpreter)

  //----------------------------------------------------------------------------
  vtkVelodyneAdvancedPacketInterpreter::vtkVelodyneAdvancedPacketInterpreter()
{
  this->CurrentFrameTracker          = new FrameTracker();
  this->MaxFrameSize                 = MEM_STEP_SIZE;
  this->CurrentArraySize             = 0;
  this->NumberOfPointsInCurrentFrame = 0;
  this->Init();
  this->DistanceResolutionM = 0.002;

  this->ReportedFactoryField1           = 0;
  this->ReportedFactoryField2           = 0;
  this->OutputPacketProcessingDebugInfo = false;
  this->UseIntraFiringAdjustment        = false;
  this->DualReturnFilter                = 0;
  this->FiringsSkip                     = 0;
  this->IsCorrectionFromLiveStream      = false;
  this->IsHDL64Data                     = false;
  this->HasDualReturn                   = false;
  this->ShouldAddDualReturnArray        = false;
  this->WantIntensityCorrection         = false;
  this->LaserSelection.resize(HDL_MAX_NUM_LASERS, true);
  this->ParserMetaData.SpecificInformation =
    std::make_shared<VelodyneSpecificFrameInformation>();
}

//------------------------------------------------------------------------------
vtkVelodyneAdvancedPacketInterpreter::~vtkVelodyneAdvancedPacketInterpreter()
{
  delete this->CurrentFrameTracker;
}

//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::ProcessPacket(
  const unsigned char * data,
  unsigned int dataLength)
{
  decltype(dataLength) index = 0;
  PayloadHeader const * payloadHeader =
    reinterpretCastWithChecks<PayloadHeader>(data, dataLength, index);
  if (payloadHeader == nullptr)
  {
    return;
  }
  ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, payloadHeader, void())

  uint8_t distanceCount = payloadHeader->GetDistanceCount();

  // A valid distance count may be 0, in which case there are no returns
  // included in the firing and thus there is nothing to do.
  if (distanceCount == 0)
  {
    return;
  }

  uint8_t distanceSize = payloadHeader->GetDistanceSizeInBytes();
  uint8_t distanceIndex;

  auto pseq       = payloadHeader->GetPseq();
  auto iset       = payloadHeader->GetIset();
  auto dsetMask   = payloadHeader->GetDsetMask();
  auto isDsetMask = payloadHeader->IsDsetMask();

  // 64-bit PTP truncated format.
  // auto timeRef = payloadHeader->GetTref();

  size_t numberOfBytesPerFiringGroupHeader = payloadHeader->GetGlen();
  size_t numberOfBytesPerFiringHeader      = payloadHeader->GetFlen();
  size_t numberOfBytesPerFiringReturn =
    payloadHeader->GetNumberOfBytesPerFiringReturn();
  size_t numberOfBytesPerFiring = payloadHeader->GetNumberOfBytesPerFiring();

  // Skip optional extension headers.
  auto nxhdr = payloadHeader->GetNxhdr();
  while (nxhdr != 0)
  {
    ExtensionHeader const * extensionHeader =
      reinterpretCastWithChecks<ExtensionHeader>(data, dataLength, index);
    if (extensionHeader == nullptr)
    {
      return;
    }
    ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, extensionHeader, void())
    nxhdr = extensionHeader->GetNxhdr();
  }

  // The included distance types are specified by a bit mask (if DSET included
  // it), for example 0110 indicates the presence of distance type 0100 and
  // 0010. To display this information, we need to retrieve the type below in
  // the firing loop. To avoid redundant bit calculations to retrieve the nth
  // bit from the mask, we can define a vector that we can easily index below
  // instead. The order of the distance types is determined by their bit values
  // in the mask per the standard so this is safe.

  // To do this, we start with the DSET mask and determine the value of the
  // first set bit. This is the distance type of the first distance. We then
  // remove that bit from the mask by subtraction and proceed to the next bit
  // and repeat. The values are stored successively so that they can be indexed
  // below.
  decltype(dsetMask) dsetRemainingMask = dsetMask;
  decltype(dsetMask) dsetBit;
  std::vector<decltype(dsetMask)> distanceTypes;
  for (distanceIndex = 0; distanceIndex < distanceCount; ++distanceIndex)
  {
    if (isDsetMask)
    {
      dsetBit = FIRST_SET_BIT(dsetRemainingMask);
      distanceTypes.push_back(dsetBit);
      dsetRemainingMask -= dsetBit;
    }
    // DSET may specify a count instead of a mask, in which case the distance
    // types are not specified. Use 0 in that case, which has no meaning in this
    // context.
    else
    {
      distanceTypes.push_back(0);
    }
  }

  // Resize the arrays if necessary.
  size_t currentArraySize = this->Points->GetNumberOfPoints();
  size_t safeArraySize    = this->NumberOfPointsInCurrentFrame +
                         payloadHeader->MaximumNumberOfPointsPerPacket();
  if (currentArraySize < safeArraySize)
  {
    this->SetNumberOfItems(safeArraySize);
  }

  // Loop through firing groups until a frame shift is detected. The number of
  // firings in each group is variable so we need to step through all of them to
  // get to the startPosition calculated by PreProcessPacket.
  size_t loopCount = 0;
  while (index < dataLength)
  {
    FiringGroupHeader const * firingGroupHeader =
      reinterpretCastWithChecks<FiringGroupHeader>(data, dataLength, index);
    if (firingGroupHeader == nullptr)
    {
      return;
    }
    // The payload header checks above ensure that this value is non-zero and
    // that the loop will therefore eventually terminate.
    index += numberOfBytesPerFiringGroupHeader;

    // Skip the firings and jump to the next firing group header.
    if (
      (loopCount++) <
      reinterpret_cast<VelodyneSpecificFrameInformation *>(
        this->ParserMetaData.SpecificInformation.get())
        ->FiringToSkip)
    {
      index += numberOfBytesPerFiring * firingGroupHeader->GetFcnt();
      continue;
    }

    bool isNewFrame =
      this->CurrentFrameTracker->Update(payloadHeader, firingGroupHeader);
    if (isNewFrame)
    {
      this->SplitFrame();
    }

    auto timeFractionOffset         = firingGroupHeader->GetToffs();
    auto coChannelSpan              = firingGroupHeader->GetFspn();
    auto coChannelTimeFractionDelay = firingGroupHeader->GetFdly();
    double verticalAngleInDegrees = firingGroupHeader->GetVerticalDeflection();
    auto azimuth                  = firingGroupHeader->GetAzm();
    double azimuthInDegrees       = firingGroupHeader->GetAzimuth();
    auto numberOfFirings          = firingGroupHeader->GetFcnt();

    for (size_t i = 0; i < numberOfFirings; ++i)
    {
      // TODO
      // This assumes that the spans are returned in order in the firing group.
      // Check that this is the case. If not, determine how to handle this (by
      // using the channe number?).
      uint32_t channelTimeFractionOffset =
        timeFractionOffset + (coChannelTimeFractionDelay * (i / coChannelSpan));

      FiringHeader const * firingHeader =
        reinterpretCastWithChecks<FiringHeader>(data, dataLength, index);
      if (firingHeader == nullptr)
      {
        return;
      }
      index += numberOfBytesPerFiringHeader;

      auto channelNumber = firingHeader->GetLcn();
      // only process point when the laser is selected
      if (!this->LaserSelection[static_cast<int>(channelNumber)])
      {
        continue;
      }

      // auto firingMode = firingHeader->GetFm();
      // auto firingModeString = toString(firingMode);
      auto power = firingHeader->GetPwr();
      auto noise = firingHeader->GetNf();
      // Status is also an enum and requires a string conversion.
      // auto status = firingHeader->GetStat();
      // auto statusString = toString(status);

      double correctedVerticalAngle =
        verticalAngleInDegrees +
        this->laser_corrections_[channelNumber].verticalCorrection;

      for (distanceIndex = 0; distanceIndex < distanceCount; ++distanceIndex)
      {
        FiringReturn firingReturn(data + index);
        index += numberOfBytesPerFiringReturn;

        uint32_t distance = firingReturn.GetDistance<uint32_t>(distanceSize);
        if (this->IgnoreZeroDistances && distance == 0)
        {
          continue;
        }

        double position[3];
        this->ComputeCorrectedValues(
          azimuth, verticalAngleInDegrees, channelNumber, position, distance);

        // Check if the point should be cropped out.
        if (this->shouldBeCroppedOut(position, azimuthInDegrees))
        {
          continue;
        }

        auto arrayIndex = this->NumberOfPointsInCurrentFrame++;
        this->Points->SetPoint(arrayIndex, position);

//! @brief Convencience macro for setting info array values.
#define VAPI_SET_VALUE(my_array, value)                                        \
  this->INFO_##my_array->SetValue(arrayIndex, value);

        VAPI_SET_VALUE(Xs, position[0])
        VAPI_SET_VALUE(Ys, position[1])
        VAPI_SET_VALUE(Zs, position[2])
        VAPI_SET_VALUE(Azimuths, azimuthInDegrees)
        VAPI_SET_VALUE(Distances, distance)
        VAPI_SET_VALUE(DistanceTypes, distanceTypes[distanceIndex])
        VAPI_SET_VALUE(Pseqs, pseq)
        VAPI_SET_VALUE(ChannelNumbers, channelNumber)
        VAPI_SET_VALUE(TimeFractionOffsets, channelTimeFractionOffset)
        VAPI_SET_VALUE(Powers, power)
        VAPI_SET_VALUE(Noises, noise)
        VAPI_SET_VALUE(VerticalAngles, correctedVerticalAngle)

        //! @brief Convenience macro for setting intensity values
#define VAPI_INSERT_INTENSITY(my_array, iset_flag)                             \
  this->INFO_##my_array->SetValue(                                             \
    arrayIndex,                                                                \
    (iset & (ISET_##iset_flag)) ? firingReturn.GetIntensity<uint32_t>(         \
                                    distanceSize, iset, (ISET_##iset_flag))    \
                                : 0);

        // TODO: Make the inclusion of these columns fully optional at runtime.

        // Add additional values here when ISET is expanded in future versions.
        VAPI_INSERT_INTENSITY(Reflectivities, REFLECTIVITY)
        VAPI_INSERT_INTENSITY(Intensities, INTENSITY)
        VAPI_INSERT_INTENSITY(Confidences, CONFIDENCE)
      }
    }
  }
}
//------------------------------------------------------------------------------
bool
vtkVelodyneAdvancedPacketInterpreter::IsLidarPacket(
  unsigned char const * data,
  unsigned int dataLength)
{
  decltype(dataLength) index = 0;

  // This checks that PayloadHeader's IsValid function, which in turn checks
  // that the version is 1 and that expected lengths are consistent.
  PayloadHeader const * payloadHeader =
    reinterpretCastWithChecks<PayloadHeader>(data, dataLength, index);
  if ((payloadHeader == nullptr) || (payloadHeader->GetHlen() > dataLength))
  {
    return false;
  }
  ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, payloadHeader, false);

  auto nxhdr = payloadHeader->GetNxhdr();
  while (nxhdr != 0)
  {
    ExtensionHeader const * extensionHeader =
      reinterpretCastWithChecks<ExtensionHeader>(data, dataLength, index);
    if (extensionHeader == nullptr)
    {
      return false;
    }
    ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, extensionHeader, false);
    nxhdr = extensionHeader->GetNxhdr();
  }

  size_t numberOfBytesPerFiring = payloadHeader->GetNumberOfBytesPerFiring();
  size_t numberOfBytesPerFiringGroupHeader = payloadHeader->GetGlen();

  while (index < dataLength)
  {
    FiringGroupHeader const * firingGroupHeader =
      reinterpretCastWithChecks<FiringGroupHeader>(data, dataLength, index);
    if (firingGroupHeader == nullptr)
    {
      return false;
    }
    // TODO
    // Add firing header checks if necessary here. See ProcessPacket for an
    // example of how to loop over each firing and advance the index.
    index += (numberOfBytesPerFiring * firingGroupHeader->GetFcnt()) +
             numberOfBytesPerFiringGroupHeader;
  }

  // return true;
  return index == dataLength;
}

//------------------------------------------------------------------------------
/*!
 * @brief         Initialize an array for datapoint attributes and add it to the
 *                polyData.
 * @tparam        T                The type of the array. This is templated so
 *                                 that the caller does not need to consider the
 *                                 type, which may change with the
 *                                 specification.
 * @param[in,out] array            The input array.
 * @param[in]     numberOfElements The number of elements that the array must be
 *                                 able to hold after initialization.
 * @param[out]    polyData         The PolyData instance to which the array
 *                                 should be added.
 */
template <typename T>
inline void
InitializeDataArrayForPolyData(
  T & array,
  char const * name,
  vtkIdType numberOfElements,
  vtkPolyData * polyData)
{
  array = T::New();
  array->SetNumberOfValues(numberOfElements);
  array->SetName(name);
  polyData->GetPointData()->AddArray(array);
}

//------------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData>
vtkVelodyneAdvancedPacketInterpreter::CreateNewEmptyFrame(
  vtkIdType numberOfPoints,
  vtkIdType prereservedNumberOfPoints)
{
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  this->UpdateMaxFrameSize(numberOfPoints);

  // Points.
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(this->MaxFrameSize);
  // Same name as vtkVelodyneHDLReader.
  points->GetData()->SetName("Points_m_XYZ");

  // Replace the old points.
  this->Points = points.GetPointer();

  // Point the polyData to the points.
  polyData->SetPoints(points.GetPointer());

  // Replace and initialize all of the associated data arrays.

//! @brief Convencience macro for initializing info arrays.
#define VAPI_INIT_INFO_ARR(arr_name, disp_name)                                \
  InitializeDataArrayForPolyData(                                              \
    this->INFO_##arr_name, disp_name, this->MaxFrameSize, polyData);

  VAPI_INIT_INFO_ARR(Xs, "X")
  VAPI_INIT_INFO_ARR(Ys, "Y")
  VAPI_INIT_INFO_ARR(Zs, "Z")
  VAPI_INIT_INFO_ARR(Distances, "raw_distance")
  VAPI_INIT_INFO_ARR(DistanceTypes, "distance_type")
  VAPI_INIT_INFO_ARR(Azimuths, "azimuth")
  VAPI_INIT_INFO_ARR(VerticalAngles, "vertical_angle") /*
VAPI_INIT_INFO_ARR(DistanceTypeStrings  , "Distance Type")
VAPI_INIT_INFO_ARR(FiringModeStrings    , "FiringMode")
VAPI_INIT_INFO_ARR(StatusStrings        , "Status")*/
  VAPI_INIT_INFO_ARR(Intensities, "intensity")
  VAPI_INIT_INFO_ARR(Confidences, "confidence")
  VAPI_INIT_INFO_ARR(Reflectivities, "reflectivity")
  VAPI_INIT_INFO_ARR(ChannelNumbers, "logical_channel_number")
  VAPI_INIT_INFO_ARR(TimeFractionOffsets, "time_fraction_offset")
  VAPI_INIT_INFO_ARR(Powers, "power")
  VAPI_INIT_INFO_ARR(Noises, "noise")
  VAPI_INIT_INFO_ARR(Pseqs, "packet_sequence_number")

  this->NumberOfPointsInCurrentFrame = 0;
  this->CurrentArraySize             = numberOfPoints;
  return polyData;
}

//------------------------------------------------------------------------------
// vtkSmartPointer<vtkPolyData>
// vtkVelodyneAdvancedPacketInterpreter::PreparePolyData()
// {
//   vtkSmartPointer<vtkPolyData> polyData =
//   vtkSmartPointer<vtkPolyData>::New(); polyData->SetPoints(this->Points);
//   auto pointData = polyData->GetPointData();
//
// #define VAPI_ADD_ARRAY(index, data, array) \
//   pointData->AddArray(array);
//
//   // "data" is an unused placeholder
//   VAPI_FOREACH_INFO_ARRAY(VAPI_ADD_ARRAY, data)
//
//   return polyData;
// }

//------------------------------------------------------------------------------
// TODO: Revisit this if the frequency still needs to be calculated here.
bool
vtkVelodyneAdvancedPacketInterpreter::SplitFrame(bool force)
{
  auto numberOfAllocatedPoints = this->Points->GetNumberOfPoints();
  // Update the MaxId to the current number of points.
  this->SetNumberOfItems(this->NumberOfPointsInCurrentFrame);
  // this->CurrentFrame->Modified();
  bool wasSplit = this->vtkLidarPacketInterpreter::SplitFrame(force);
  // If the frame was split then CreateNewEmptyDataFrame was called and the
  // array sizes have already been adjusted. If not, we need to reset the MaxId
  // to allow for further insertions.
  if (!wasSplit)
  {
    this->SetNumberOfItems(numberOfAllocatedPoints);
  }
  return wasSplit;
}

//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::ResetCurrentFrame()
{
  this->CurrentFrame = this->CreateNewEmptyFrame(0);
  this->CurrentFrameTracker->Reset();
  this->Frames.clear();
}

//------------------------------------------------------------------------------
bool
vtkVelodyneAdvancedPacketInterpreter::PreProcessPacket(
  const unsigned char * data,
  unsigned int dataLength,
  fpos_t filePosition,
  double packetNetworkTime,
  std::vector<FrameInformation> * frameCatalog)
{
  this->ParserMetaData.FilePosition           = filePosition;
  this->ParserMetaData.FirstPacketNetworkTime = packetNetworkTime;
  //! @todo
  //  this->ParserMetaData.FirstPacketDataTime = packetNetworkTime;
  auto * velFrameInfo =
    reinterpret_cast<VelodyneSpecificFrameInformation *>(
      this->ParserMetaData.SpecificInformation.get());
  //  if (dataPacket->gpsTimestamp < this->lastGpsTimestamp)
  //  {
  //    velFrameInfo->NbrOfRollingTime++;
  //  }

  decltype(dataLength) index = 0;
  PayloadHeader const * payloadHeader =
    reinterpretCastWithChecks<PayloadHeader>(data, dataLength, index);
  if ((payloadHeader == nullptr) || (payloadHeader->GetDistanceCount() == 0))
  {
    return false;
  }
  ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, payloadHeader, false)

  // Skip optional extension headers.
  auto nxhdr = payloadHeader->GetNxhdr();
  while (nxhdr != 0)
  {
    ExtensionHeader const * extensionHeader =
      reinterpretCastWithChecks<ExtensionHeader>(data, dataLength, index);
    if (extensionHeader == nullptr)
    {
      return false;
    }
    ADVANCE_INDEX_BY_HLEN_OR_RETURN(dataLength, index, extensionHeader, false)
    nxhdr = extensionHeader->GetNxhdr();
  }

  // Loop through firing groups until a frame shift is detected.
  size_t numberOfBytesPerFiringGroupHeader = payloadHeader->GetGlen();
  size_t numberOfBytesPerFiring = payloadHeader->GetNumberOfBytesPerFiring();
  int firingCount               = 0;
  bool isNewFrame               = false;
  while (index < dataLength)
  {
    FiringGroupHeader const * firingGroupHeader =
      reinterpretCastWithChecks<FiringGroupHeader>(data, dataLength, index);
    if (firingGroupHeader == nullptr)
    {
      return isNewFrame;
    }
    // The payload header checks above ensure that this value is non-zero and
    // that the loop will therefore eventually terminate.
    isNewFrame =
      this->CurrentFrameTracker->Update(payloadHeader, firingGroupHeader);
    if (isNewFrame)
    {
      velFrameInfo->FiringToSkip = firingCount;
      frameCatalog->push_back(this->ParserMetaData);
      // Create a copy of the current meta data state
      // at a different memory location than the one
      // added to the catalog
      return isNewFrame;
    }
    firingCount++;
    index += (numberOfBytesPerFiring * firingGroupHeader->GetFcnt()) +
             numberOfBytesPerFiringGroupHeader;
  }
  return isNewFrame;
}

//------------------------------------------------------------------------------
// Code from the legacy packet format interpreter.
//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::InitTrigonometricTables()
{
  if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
  {
    cos_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    sin_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
    {
      double rad           = degreesToRadians(i / 100.0);
      cos_lookup_table_[i] = std::cos(rad);
      sin_lookup_table_[i] = std::sin(rad);
    }
  }
}

//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::PrecomputeCorrectionCosSin()
{
  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
  {
    HDLLaserCorrection & correction = laser_corrections_[i];
    correction.cosVertCorrection =
      std::cos(degreesToRadians(correction.verticalCorrection));
    correction.sinVertCorrection =
      std::sin(degreesToRadians(correction.verticalCorrection));
    correction.cosRotationalCorrection =
      std::cos(degreesToRadians(correction.rotationalCorrection));
    correction.sinRotationalCorrection =
      std::sin(degreesToRadians(correction.rotationalCorrection));
    correction.sinVertOffsetCorrection =
      correction.verticalOffsetCorrection * correction.sinVertCorrection;
    correction.cosVertOffsetCorrection =
      correction.verticalOffsetCorrection * correction.cosVertCorrection;
  }
}

//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::Init()
{
  this->InitTrigonometricTables();
  // this->SensorTransform->Identity()
  this->ResetCurrentFrame();
}

//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::LoadCalibration(
  const std::string & filename)
{
  boost::property_tree::ptree pt;
  try
  {
    read_xml(filename, pt, boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (boost::exception const &)
  {
    vtkGenericWarningMacro(
      "LoadCalibration: error reading calibration file: " << filename);
    return;
  }
  // Read distLSB if provided
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type & v,
    pt.get_child("boost_serialization.DB"))
  {
    if (v.first == "distLSB_")
    { // Stored in cm in xml
      DistanceResolutionM = atof(v.second.data().c_str()) / 100.0;
    }
  }

  int i, j;
  i = 0;
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type & p,
    pt.get_child("boost_serialization.DB.colors_"))
  {
    if (p.first == "item")
    {
      j = 0;
      BOOST_FOREACH (
        boost::property_tree::ptree::value_type & v, p.second.get_child("rgb"))
        if (v.first == "item")
        {
          std::stringstream ss;
          double val;
          ss << v.second.data();
          ss >> val;

          XMLColorTable[i][j] = val;
          j++;
        }
      i++;
    }
  }

  int enabledCount = 0;
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type & v,
    pt.get_child("boost_serialization.DB.enabled_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      int test = 0;
      ss >> test;
      if (!ss.fail() && test == 1)
      {
        enabledCount++;
      }
    }
  }
  this->CalibrationReportedNumLasers = enabledCount;

  // Getting min & max intensities from XML
  int laserId = 0;
  int minIntensity[HDL_MAX_NUM_LASERS], maxIntensity[HDL_MAX_NUM_LASERS];
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type & v,
    pt.get_child("boost_serialization.DB.minIntensity_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      ss >> minIntensity[laserId];
      laserId++;
    }
  }

  laserId = 0;
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type & v,
    pt.get_child("boost_serialization.DB.maxIntensity_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      ss >> maxIntensity[laserId];
      laserId++;
    }
  }

  BOOST_FOREACH (
    boost::property_tree::ptree::value_type & v,
    pt.get_child("boost_serialization.DB.points_"))
  {
    if (v.first == "item")
    {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH (boost::property_tree::ptree::value_type & px, points)
      {
        if (px.first == "px")
        {
          boost::property_tree::ptree calibrationData = px.second;
          int index                                   = -1;
          HDLLaserCorrection xmlData;

          BOOST_FOREACH (
            boost::property_tree::ptree::value_type & item, calibrationData)
          {
            if (item.first == "id_")
              index = atoi(item.second.data().c_str());
            if (item.first == "rotCorrection_")
              xmlData.rotationalCorrection = atof(item.second.data().c_str());
            if (item.first == "vertCorrection_")
              xmlData.verticalCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrection_")
              xmlData.distanceCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrectionX_")
              xmlData.distanceCorrectionX = atof(item.second.data().c_str());
            if (item.first == "distCorrectionY_")
              xmlData.distanceCorrectionY = atof(item.second.data().c_str());
            if (item.first == "vertOffsetCorrection_")
              xmlData.verticalOffsetCorrection =
                atof(item.second.data().c_str());
            if (item.first == "horizOffsetCorrection_")
              xmlData.horizontalOffsetCorrection =
                atof(item.second.data().c_str());
            if (item.first == "focalDistance_")
              xmlData.focalDistance = atof(item.second.data().c_str());
            if (item.first == "focalSlope_")
              xmlData.focalSlope = atof(item.second.data().c_str());
            if (item.first == "closeSlope_")
              xmlData.closeSlope = atof(item.second.data().c_str());
          }
          if (index != -1 && index < HDL_MAX_NUM_LASERS)
          {
            laser_corrections_[index] = xmlData;
            // Angles are already stored in degrees in xml
            // Distances are stored in centimeters in xml, and we store meters.
            laser_corrections_[index].distanceCorrection /= 100.0;
            laser_corrections_[index].distanceCorrectionX /= 100.0;
            laser_corrections_[index].distanceCorrectionY /= 100.0;
            laser_corrections_[index].verticalOffsetCorrection /= 100.0;
            laser_corrections_[index].horizontalOffsetCorrection /= 100.0;
            laser_corrections_[index].focalDistance /= 100.0;
            laser_corrections_[index].focalSlope /= 100.0;
            laser_corrections_[index].closeSlope /= 100.0;
            if (laser_corrections_[index].closeSlope == 0.0)
              laser_corrections_[index].closeSlope =
                laser_corrections_[index].focalSlope;
            laser_corrections_[index].minIntensity = minIntensity[index];
            laser_corrections_[index].maxIntensity = maxIntensity[index];
          }
        }
      }
    }
  }

  int idx = 0;
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type & v,
    pt.get_child("boost_serialization.DB.minIntensity_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      int intensity = 0;
      ss >> intensity;
      if (!ss.fail() && idx < HDL_MAX_NUM_LASERS)
      {
        laser_corrections_[idx].minIntensity = intensity;
      }
      idx++;
    }
  }

  idx = 0;
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type & v,
    pt.get_child("boost_serialization.DB.maxIntensity_"))
  {
    std::stringstream ss;
    if (v.first == "item")
    {
      ss << v.second.data();
      int intensity = 0;
      ss >> intensity;
      if (!ss.fail() && idx < HDL_MAX_NUM_LASERS)
      {
        laser_corrections_[idx].maxIntensity = intensity;
      }
      idx++;
    }
  }

  PrecomputeCorrectionCosSin();
  this->IsCalibrated = true;
}

//------------------------------------------------------------------------------
template <typename TAzm, typename TDist>
void
vtkVelodyneAdvancedPacketInterpreter::ComputeCorrectedValues(
  TAzm const azimuth,
  double const verticalAngleInDegrees,
  size_t const correctionIndex,
  double pos[3],
  TDist & distance)
{
  double cosAzimuth, sinAzimuth, cosVertCorrection, sinVertCorrection;
  HDLLaserCorrection * correction =
    &(this->laser_corrections_[correctionIndex]);

  if (correction->rotationalCorrection == 0)
  {
    cosAzimuth = this->cos_lookup_table_[azimuth];
    sinAzimuth = this->sin_lookup_table_[azimuth];
  }
  else
  {
    // realAzimuth = azimuth/100 - rotationalCorrection
    // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
    // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
    cosAzimuth =
      this->cos_lookup_table_[azimuth] * correction->cosRotationalCorrection +
      this->sin_lookup_table_[azimuth] * correction->sinRotationalCorrection;
    sinAzimuth =
      this->sin_lookup_table_[azimuth] * correction->cosRotationalCorrection -
      this->cos_lookup_table_[azimuth] * correction->sinRotationalCorrection;
  }

  if (verticalAngleInDegrees == 0)
  {
    cosVertCorrection = correction->cosVertCorrection;
    sinVertCorrection = correction->sinVertCorrection;
  }
  else
  {
    double VerticalAngleInRadians = degreesToRadians(verticalAngleInDegrees + correction->verticalCorrection);
    cosVertCorrection = std::cos(VerticalAngleInRadians);
    sinVertCorrection = std::sin(VerticalAngleInRadians);
  }

  // double cosVertOffsetCorrection = correction->verticalOffsetCorrection * cosVertCorrection;
  double sinVertOffsetCorrection = correction->verticalOffsetCorrection * sinVertCorrection;

  // Compute the distance in the xy plane (w/o accounting for rotation)
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathemathical
   * model we used.
   */
  double distanceMRaw = distance * this->DistanceResolutionM;
  double distanceM    = distanceMRaw + correction->distanceCorrection;
  double xyDistance   = distanceM * cosVertCorrection - sinVertOffsetCorrection;

  pos[0] = xyDistance * sinAzimuth -
           correction->horizontalOffsetCorrection * cosAzimuth;
  pos[1] = xyDistance * cosAzimuth +
           correction->horizontalOffsetCorrection * sinAzimuth;
  pos[2] = distanceM * sinVertCorrection +
           correction->verticalOffsetCorrection;
}

//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::UpdateMaxFrameSize(size_t frameSize)
{
  if (frameSize > this->MaxFrameSize)
  {
    size_t difference = frameSize - this->MaxFrameSize;
    this->MaxFrameSize +=
      ((difference + (MEM_STEP_SIZE - 1)) / MEM_STEP_SIZE) * MEM_STEP_SIZE;
  }
}

//------------------------------------------------------------------------------
// Macro-based methods.
//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::ResizeArrays()
{
  size_t newSize     = this->MaxFrameSize;
  size_t currentSize = this->CurrentArraySize;
  if (newSize <= currentSize)
  {
    return;
  }

  this->Points->Resize(newSize);

#define VAPI_RESIZE(index, data, array) array->Resize(newSize);

  // "data" is an unused placeholder
  VAPI_FOREACH_INFO_ARRAY(VAPI_RESIZE, data)

  this->CurrentArraySize = newSize;
}
//------------------------------------------------------------------------------
void
vtkVelodyneAdvancedPacketInterpreter::SetNumberOfItems(size_t numberOfItems)
{
  this->UpdateMaxFrameSize(numberOfItems);
  if (numberOfItems > static_cast<size_t>(this->Points->GetNumberOfPoints()))
  {
    this->ResizeArrays();
  }

  this->Points->SetNumberOfPoints(numberOfItems);

#define VAPI_SET_NUMBER_OF_VALUES(index, data, array)                          \
  array->SetNumberOfValues(numberOfItems);

  VAPI_FOREACH_INFO_ARRAY(VAPI_SET_NUMBER_OF_VALUES, data)
}

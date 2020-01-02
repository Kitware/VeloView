#ifndef VELODYNE_APF_COMMON_H
#define VELODYNE_APF_COMMON_H

#include <boost/endian/arithmetic.hpp>
#include <boost/predef/detail/endian_compat.h> // deprecated
// #include <boost/predef/other/endian.h> // replacement

#include <vtkSetGet.h>
#include "EnumMacros.h"
#include "VelodyneInterpreterCommon.h"

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
// Info array macros.
//------------------------------------------------------------------------------
//! @ brief List of field macros for use in other macros.
#define VAPI_INFO_ARRAYS \
	(Xs)                   \
	(Ys)                   \
	(Zs)                   \
	(Distances)            \
	(RawDistances)         \
	(Azimuths)             \
	(VerticalAngles)       \
	(Confidences)          \
	(Intensities)          \
	(Reflectivities)       \
	(DistanceTypes)        \
	(ChannelNumbers)       \
	(Noises)               \
	(Powers)               \
	(Pseqs)                \
	(TimeFractionOffsets)  \
	(Timestamps)
	// (DistanceTypeStrings)

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
  ((REFLECTIVITY , (1u << 0)))
  ((INTENSITY    , (1u << 1)))
  ((CONFIDENCE   , (1u << 2)))
  ((RESERVED     , (1u << 3))),
  RESERVED
  )


//------------------------------------------------------------------------------
//! @brief Mask format to specify values in returned distance set.
// Bists 4-5 are currently reserved.
DEFINE_ENUM(
  DistanceType,
  DSET_,
  ((UNSPECIFIED      , (0)))
  ((FIRST            , (1u << 0)))
  ((STRONGEST        , (1u << 1)))
  ((SECOND_STRONGEST , (1u << 2)))
  ((LAST             , (1u << 3)))
  ((RESERVED         , (1u << 4))),
  RESERVED
)

// clang-format on

//------------------------------------------------------------------------------
/*!
 * @brief         Align a value to the pre-defined header word size.
 * @tparam        T   The type of the value to align.
 * @param[in,out] The value to align.
 */
template <typename T>
inline
void align_to_word_size(T & x)
{
  // Check that VAPI_BYTES_PER_HEADER_WORD is a power of 2 at compile time.
  constexpr bool isPowerOfTwo = VAPI_BYTES_PER_HEADER_WORD && (VAPI_BYTES_PER_HEADER_WORD & (VAPI_BYTES_PER_HEADER_WORD - 1)) == 0;
  static_assert(isPowerOfTwo, "VAPI_BYTES_PER_HEADER_WORD is not a power of 2");
  // Use bitshifts instead of the modulus operator and branching.
  T staggered = x & (VAPI_BYTES_PER_HEADER_WORD - 1);
  // Adds 0 if staggered is already 0.
  x += (VAPI_BYTES_PER_HEADER_WORD - staggered) & (VAPI_BYTES_PER_HEADER_WORD - 1);
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
  if (!objectPtr->IsValid(dataLength))
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
  if ((dataLength < index) || (hlen > (dataLength - index)))                   \
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

  //! @brief Get the number of nanoseconds represented by TREF.
  uint64_t
  GetTrefInNanoseconds() const
  {
    uint64_t ns = 0, tref = this->Tref;
    ns += (tref >> 32) * 1000000000;
    ns += (tref & 0xffffffff);
    return ns;
  }

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
    return (bitRangeValue<uint8_t>(this->Dset, 7, 1) ? 3 : 2);
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
    return SET_BITS_IN_BYTE[this->Iset];
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
   * @brief     Returns true if the packet header appears to be a valid version
   *            1 header.
   * @param[in] length The number of bytes available for casting.
   */
  inline bool
  IsValid(size_t length) const
  {
    return (
        (this->GetVer() == 1) &&
        (this->GetHlen() >= 5) &&
        (this->GetHlen() <= length) &&
        (this->GetGlen() >= 2)
      );
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

  /*!
   * @brief     Check if this extension header appears to be valid.
   * @param[in] length The number of bytes available for casting.
   */
  inline bool
  IsValid(size_t length) const
  {
    return (
        (this->GetHlen() > 0) &&
        (this->GetHlen() <= length)
      );
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

  /*!
   * @brief     Check if this extension header appears to be valid.
   * @param[in] length The number of bytes available for casting.
   */
  inline bool
  IsValid(size_t vtkNotUsed(length)) const
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

  /*!
   * @brief     Check if this extension header appears to be valid.
   * @param[in] length The number of bytes available for casting.
   */
  inline bool
  IsValid(size_t vtkNotUsed(length)) const
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
  uint8_t const MaxLength;

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
    i += bytesPerDistance;
    return ((i < this->MaxLength) ? this->Data[i] : 0);
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
    uint8_t i = bytesPerDistance;

    // Count the number of bits set before the requested one. `mask` constains a
    // single set bit. By decrementing the value, we obtain a mask over all
    // preceding bits which we can then count to get the offset of the requested
    // intensity.
    uint8_t mask = static_cast<uint8_t>(type);
    if (mask)
    {
      --mask;
      i += SET_BITS_IN_BYTE[iset & mask];
    }
    return ((i < this->MaxLength) ? this->Data[i] : 0);
  }

  FiringReturn(uint8_t const * data, uint8_t const maxLen) : Data(data), MaxLength(maxLen) {};
};
#pragma pack(pop)

#endif // VELODYNE_APF_COMMON_H

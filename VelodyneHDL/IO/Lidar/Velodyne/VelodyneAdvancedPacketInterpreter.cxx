#include "VelodyneAdvancedPacketInterpreter.h"

#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>

#include <type_traits>
#include <boost/preprocessor.hpp>

//------------------------------------------------------------------------------
// Macros.
//------------------------------------------------------------------------------
//! @brief Lengths in the headers are given in units of 32-bit words.
#define BYTES_PER_HEADER_WORD 4u;

//------------------------------------------------------------------------------
// Accessor macros.
//------------------------------------------------------------------------------
//! @brief Simple getter for uninterpretted values.
#define GET_RAW(attr) decltype(auto) Get ## attr const { return this->attr; }

//! @brief Get a const reference.
#define GET_CONST_REF(attr) auto const & Get ## attr const { return this->attr; }

//! @brief Getter for enum values that are stored as raw values internally.
#define GET_ENUM(enumtype, attr) type Get ## attr const { return to ## enumtype(this->attr); }

//! @brief Get a header length in bytes.
#define GET_LENGTH(attr) decltype(auto) Get ** attr const { return this->attr * BYTES_PER_HEADER_WORD; }

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
  char const * toString(name x)                                                         \
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
  name to ## name(T x)                                                 \
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
// Global constants.
//------------------------------------------------------------------------------
//! @brief Lookup table for the number of set bits in a byte.
static uint8_t const SET_BITS_IN_BYTE[0x100] = {
  0, 1, 1, 2,   1, 2, 2, 3,   1, 2, 2, 3,   2, 3, 3, 4,   1, 2, 2, 3,   2, 3, 3, 4,   2, 3, 3, 4,   3, 4, 4, 5
  1, 2, 2, 3,   2, 3, 3, 4,   2, 3, 3, 4,   3, 4, 4, 5,   2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6
  1, 2, 2, 3,   2, 3, 3, 4,   2, 3, 3, 4,   3, 4, 4, 5,   2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6
  2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,   3, 4, 4, 5,   4, 5, 5, 6,   4, 5, 5, 6,   5, 6, 6, 7
  1, 2, 2, 3,   2, 3, 3, 4,   2, 3, 3, 4,   3, 4, 4, 5,   2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6
  2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,   3, 4, 4, 5,   4, 5, 5, 6,   4, 5, 5, 6,   5, 6, 6, 7
  2, 3, 3, 4,   3, 4, 4, 5,   3, 4, 4, 5,   4, 5, 5, 6,   3, 4, 4, 5,   4, 5, 5, 6,   4, 5, 5, 6,   5, 6, 6, 7
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

//! @brief Model identification code.
DEFINE_ENUM(
  ModelIdentificationCode,
  MIC_,
  ((RESERVED , 0))
  ((VLP16    , 1))
  ((VLP16_HD , 2))
  ((VLP32A   , 3))
  ((VLP32B   , 4)) ,
  RESERVED)

//------------------------------------------------------------------------------
//! @brief Horizontal direction of Lidar.
DEFINE_ENUM(
  HorizontalDirection,
  HDIR_,
  ((CLOCKWISE         , 0))
  ((COUNTER_CLOCKWISE , 1))
  HD_CLOCKWISE
)

//------------------------------------------------------------------------------
//! @brief Vertical direction of Lidar.
DEFINE_ENUM(
  VerticalDirection,
  VDIR_
  ((UP   , 0))
  ((DOWN , 1))
  VD_UP
)

//------------------------------------------------------------------------------
//! @brief Firing mode.
DEFINE_ENUM(
  FiringMode,
  FM_
  ((PASSIVE  , 0))
  ((NORMAL   , 1))
  ((RESERVED , 2))
  ((INDEX    , 15))
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
  ((RESERVED             , 3))
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
  ((RESERVED     , (1u << 2)))
  RESERVED
}

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
  ((RESERVED         , (1u << )))
  RESERVED
}

//------------------------------------------------------------------------------
// Convenience functions.
//------------------------------------------------------------------------------
/*!
  * @brief     Convert degrees to radians.
  * @param[in] degrees The input value in degrees.
  * @return    The input value converted to radians.
  */
inline double degreesToRadians(double degrees)
{
  return (degrees * vtkMath::Pi()) / 180.0;
}

//----------------------------------------------------------------------------
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
template <typename T>
T indexOfBit(T value, T bit)
{
  T precedingBits = (bit - 1) & value;
  // Count the bits set before the target bit. This will be the index of the set
  // bit.
  if (sizeof(T) > BITS_PER_BYTE)
  {
    T byteMask = (1 << BITS_PER_BYTE) - 1;
    T index = 0;
    while(precedingBits)
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

//----------------------------------------------------------------------------
/*!
 * @brief     Return the first set bit in a value.
 * @param[in] x The value.
 * @return    A value with the first set bit.
 */
template <typename T>
inline
T firstSetBit(T x)
{
  return (x - (x & (x - 1)));
}

//----------------------------------------------------------------------------
/*!
 * @brief         Set a value from a byte sequence.
 * @todo          Add endianness detection and optimize accordingly.
 * @param[in]     bytes         The array of input bytes.
 * @param[in,out] index         The index of the first byte to read.
 * @param[out]    value         The output value.
 * @param[in]     numberOfBytes Number of bytes to use for the value.
 */
template <typename IndexT, typename ValueT>
inline
void setFromBytes(
  uint8_t const * bytes,
  IndexT & index,
  ValueT & value,
  size_t numberOfBytes = sizeof(T)
)
{
  IndexT stop = index + numberOfBytes;
  value = bytes[index++];
  while (index < stop)
  {
    value = bytes[index++] + (value * (1 << sizeof(uint8_t)));
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief      Set a target value from a range of bits in another value.
 * @tparam     TS The source value type.
 * @tparam     TD The destination value type.
 * @param[in]  source      The source value from which to set the destination
 *                         value.
 * @param[in]  offset      The offset of the least significant bit.
 * @param[in]  number      The number of bits to set.
 * @param[out] destination The destination value to set.
 */
template <typename TS, typename TD>
inline
void setFromBits(TS & source, uint8_t offset, uint8_t number, TD & destination)
{
  T const mask = (1 << number) - 1;
  target = (value >> offset) & mask;
}

/*!
 * @copydoc setFromBits
 * @brief   Variadic overload to set multiple destination values from the same
 *          bytes. Extra values are passed as triples of offset, number and
 *          destination.
 * @todo    Check if this has any noticeable performance overhead.
 */
template <typename TS, typename TD, typename... RemainingArgs>
inline
void setFromBits(TS & source, uint8_t offset, uint8_t number, TD & destination, RemainingArgs... remainingArgs)
{
  setFromBits(source, offset, number, destination);
  setFromBits(source, remainingArgs);
}

//------------------------------------------------------------------------------
// BlockData
//------------------------------------------------------------------------------
//! @brief Data used to track word-aligned blocks in the packet data.
struct BlockData
{
public:
  size_t Index;
  size_t WordSize;

  BlockData(size_t index, size_t wordSize)
    : Index = index, WordSize = wordSize
  {
  }
}

//------------------------------------------------------------------------------
// Packetdata
//------------------------------------------------------------------------------
/*!
 * @brief Convencience struct for processing packet data.
 *
 * This transparently manages an index to the next unprocessed byte and provides
 * methods to set a value from a single byte, several bytes, or single- and
 * multi-byte ranges of bits.
 */
class PacketData
{
private:
  //! @brief The raw data.
  uint8_t const * Data;

  //! @brief The number of bytes in the raw data.
  size_t Length;

  //! @brief The index of the next byte to process.
  size_t Index;

  //! @brief Tracked data for aligning blocks on word limits.
  std::vector<BlockData> Blocks;

public:
  //@{
  //! @brief Getters.
  GET_RAW(Data);
  GET_RAW(Length);
  GET_RAW(Index);
  //@}

  /*!
   * @brief Mark the beginning of a block to enabled automatic word alignment
   *        when it ends.
   * @param[in] wordSize The word size on which to align the block.
   *
   * This should be called before any values are read from the block. When all
   * values have been read, call EndBlock().
   */
  void BeginBlock(size_t wordSize = BYTES_PER_HEADER_WORD)
  {
    this->Blocks.push_back(BlockData(this->Index, wordSize));
  }

  //@{
  /*!
   * @brief End a block by skipping any expected padding required for word
   *        alignment.
   *
   * BeginBlock() must be called first. If no argument is given, the index will
   * be set to the next word boundary. An argument is interpretted as the block
   * length in bytes and the index is advanced from the start of the block by
   * this value. An error will be thrown if the length is shorter than the
   * number of consumed bytes since BeginBlock() was called. 
   */
  void EndBlock()
  {
    auto blockData = this->Blocks.pop_back();
    auto wordRemainder = (this->Index - blockData.Index) % blockData.WordSize;
    if (wordRemainder > 0)
    {
      this->Index += (blockData.WordSize - wordRemainder);
    }
  }
  void EndBlock(size_t length)
  {
    auto blockData = this->Blocks.pop_back();
    if ((this->Index - blockData.Index) > length)
    {
      raise std::length_error("EndBlock(): block size length is less than the number of consumed bytes since BeginBlock() was called");
    }
    this->Index = this->blockData.Index + length;
  }
  //@}
  
  /*!
   * @brief Reset the index to the beginning of the current block and remove it.
   */
  void ResetBlock()
  {
    auto blockData = this->Blocks.pop_back();
    this->Index = blockData.Index;
  }
  
  //! @brief Get the number of bytes remaining from the current index to the end
  //         of the data as defined by the length.
  size_t GetRemainingLength()
  {
    return (this->Length > this-Index) ? (this->Length - this->Index) : 0;
  }

  /*!
   * @brief      Set a 1-byte value.
   * @tparam     T     The type of the value to set.
   * @param[out] value The value to set.
   */
  template <typename T>
  void SetFromByte(T & value)
  {
    value = this->Data[(this->Index)++];
  }

  /*!
   * @brief      Set a multi-byte value (wraps setFromBytes).
   * @tparam     T             The type of the value to set.
   * @param[out] value         The value to set.
   * @param[in]  numberOfBytes The number of bytes to consume. If not given, the
   *                           size of the value type will be used.
   */
  template <typename T>
  void SetFromBytes(T & value, size_t numberOfBytes = sizeof(T))
  {
    setFromBytes(this->Data, this->Index, value, numberOfBytes);
  }

  /*!
   * @brief  Set values from bit sequences that do not align with bytes.
   * @tparam numberOfBytes   The number of bytes to consume from the input data.
   *                         Values larger than 4 are not supported.
   * @tparam PassthroughArgs All arguments to pass through to SetFromBits.
   *
   * This is a wrapper around SetFromBits with automatic deduction of the
   * smallest type required to hold the number of bytes to process.
   */
	// TODO: add insert to fail if numberOfBytes is greater than 8
  template <uint8_t numberOfBytes, typename... PassthroughArgs>
  void SetFromBits(PassthroughArgs... passthroughArgs)
  {
    // Select a placeholder type large enough to hold the required value. All
    // bits are counted from the least significant bit so bits in the padding
    // byte(s), if any, will be ignored.
    typename std::conditional<
      (numberOfBytes == 1),
      uint8_t,
      typename std::conditional<
        (numberOfBytes == 2),
        uint16_t,
        typename std::conditional<(numberOfBytes <= 4), uint32_t, uint64_t>:: type
      >::type
    >::type placeholder;

    this->SetFromBytes(placeholder, numberOfBytes);
    SetFromBits(placeholder, passthroughArgs);
  }

  /*!
   * @brief      Copy a sequence of bytes (wrapper around std::memcpy)..
   * @param[out] data   The destination to which to copy the data.
   * @param[in]  length The number of bytes to copy.
   */
  void CopyBytes(std::vector<uint8_t> & data, size_t length)
  {
    data.reserve(length);
    std::memcpy(data.(), this->Data[this->Index], length);
    this->Index += length;
  }

  /*!
   * @brief     Skip a given number of bytes.
   * @param[in] length The number of bytes to skip.
   */
  void SkipBytes(size_t length)
  {
    this->Index += length;
  }
}

//------------------------------------------------------------------------------
// PayloadHeader
//------------------------------------------------------------------------------
/*!
 * @brief Payload header of the VLP Advanced data packet format.
 *
 * All lengths count 32-bit words, e.g. a Glen value of 4 indicates that the
 * firing group header consists of 4 32-bit words. The raw values are converted
 * to counts with the BYTES_PER_HEADER_WORD constant.
 */
class PayloadHeader
{
private:
  //! @brief Protocol version.
  uint8_t Ver : 4;

  //! @brief Header length (min: 6)
  uint8_t Hlen : 4;

  //! @brief Next header type.
  uint8_t Nxhdr;

  //! @brief Firing group header length.
  uint8_t Glen : 4;

  //! @brief Firing header length.
  uint8_t Flen : 4;

  //! @brief Model Identification Code
  uint8_t Mic;

  //! @brief Time status (TBD).
  uint8_t Tstat;

  //! @brief Distance set.
  uint8_t Dset;

  //! @brief Intensity set.
  uint8_t Iset;

  //! @brief Time reference.
  uint64_t Tref;

  //! @brief Payload sequence number.
  uint32_t Pseq;

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * HLEN, GLEN and FLEN are returned in bytes, not the number of 32-bit words.
   */
  GET_RAW(Ver)
  GET_LENGTH(Hlen)
  GET_RAW(Nxhdr)
  GET_LENGTH(Glen)
  GET_LENGTH(Flen)
  GET_ENUM(ModelIdentificationCode, Mic);
  GET_RAW(Tstat)
  GET_RAW(Dset)
  GET_RAW(Iset)
  GET_RAW(Tref)
  GET_RAW(Pset)
  //@}


  //! @brief The number of bytes per value in a return.
  uint8_t GetDsetEncodingSizeInBytes const ()
  {
    return ((this->Dset & (1 << 7)) ? 3 : 2);
  }

  //! @brief True if the distance set includes a mask, false if a count.
  bool IsDsetMask const ()
  {
    return ! (this->Dset & (1 << 6));
  }

  //! @brief The mask or count, depending on the return value of isDsetMask.
  uint8_t GetDsetMask const ()
  {
    return (this->Dset & ((1 << 6) - 1));
  }

  //! @brief Get the number of distances in each firing group.
  uint8_t GetDistanceCount const ()
  {
    // The "mask" is actually a count if IsDsetMask is false.
    auto mask = this->GetDsetMask();
    return (this->IsDsetMask()) ? SET_BITS_IN_BYTE[mask] : mask;

  }

  //! @brief Get the number of intensities in each firing.
  uint8_t GetIntensityCount const ()
  {
    return SET_BITS_IN_BYTE[this->Iset];
  }

  //! @brief The the number of bytes per firing return.
  size_t GetNumberOfBytesPerFiringReturn const ()
  {
    size_t icount = this->GetIntensityCount();
    size_t bytesPerValue = this->.GetBytesPerDistance();
    return icount * bytesPerValue;
  }

  //! @brief The the number of bytes per firing.
  size_t GetNumberOfBytesPerFiring const ()
  {
    size_t dcount = this->GetDistanceCount();
    size_t bytesPerReturn = this->GetNumberOfBytesPerFiringReturn();
    return dcount * bytesPerReturn;
  }

  /*!
   * @brief         Construct a PayloadHeader.
   * @param[in,out] packetData The packet data from which to parse the header.
   */
  PayloadHeader(PacketData & packetData)
  {
    // For word alignment.
    packetData.BeginBlock();
    packetData.SetFromBits<1>(
      4, 4, this->Ver,
      0, 4, this->Hlen
    );

    // Use GetHlen() to get the count in bytes.
    if (this->GetHlen() > packetData.GetRemainingLength())
    {
      raise std::length_error("data does not contain enough bytes for header");
    }

    packetData.SetFromByte(this->Nxhdr);
    packetData.SetFromBits<1>(
      4, 4, this->Glen,
      0, 4, this->Flen
    );
    packetData.SetFromByte(this->Mic);
    packetData.SetFromByte(this->Tstat);
    packetData.SetFromByte(this->Dset);
    packetData.SetFromByte(this->Iset);
    packetData.SetFromBytes(this->Tref);
    packetData.SetFromBytes(this->Pseq);
    // For word alignment.
    packetData.EndBlock(this->GetHlen());
  }
}


//------------------------------------------------------------------------------
// ExtensionHeader
//------------------------------------------------------------------------------
/*!
 * @brief  Extension header of the VLP Advanced data packet format.
 *
 * This should be subclassed to handle different types of extensions as
 * specified by the NXHDR field of the payload header.
 * 
 * @tparam loadData If true, load the extension data, otherwise skip it.
 */
template <bool loadData>
class ExtensionHeader
{
private:
  //! @brief The length of the extension header.
  uint8_t HLen;

  //! @brief Nxhdr The next header type (same as PayloadHeader).
  uint8_t Nxhdr;

  /*!
    * @brief Extension header data value.
    *
    * The data field must end on a 32-bit boundary. The format of the data is
    * extension-specific and determined by the NXHDR value of the previous
    * header (either the payload header or a preceding extension header).
    */
  std::vector<uint8_t> Data;

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * HLEN is returned in bytes, not the number of 32-bit words.
   */
  GET_LENGTH(Hlen)
  GET_RAW(Nxhdr)
  GET_CONST_REF(Data)
  //@}

  /*!
   * @brief         Construct a ExtensionHeader.
   * @param[in]     data The packet data.
   * @param[in,out] i    The offset to the start of the header. The offset will
   *                     be advanced as the data is consumed.
   */
  ExtensionHeader(PacketData & packetData)
  {
    packetData.SetFromByte(this->Hlen);

    auto hlen = this->GetHlen();
    if (hlen > packetData.GetRemainingLength())
    {
      raise std::length_error("data does not contain enough bytes for header");
    }

    packetData.SetFromByte(this->Nxhdr);

    auto dataLen = hlen - (sizeof(Hlen) + sizeof(Nxhdr));

    // Only copy the data if the template parameter requests it.
    if (loadData)
    {
      packetData.CopyBytes(this->Data, dataLen);
    }
    else
    {
      packetData.SkipBytes(dataLen);
    }
  }
}

//------------------------------------------------------------------------------
// FiringGroupHeader
//------------------------------------------------------------------------------
/*!
 * @brief Firing group header of the VLP Advanced data packet format.
 */
class FiringGroupHeader
{
private:
  /*!
   * @brief
   * Unsigned time fraction offset from payload timestamp to firing time
   * of the Firing Group in units of 64 ns.
   */
  uint16_t Toffs;

  //! @brief (FCNT + 1) is the number of Firings in the Firing Group.
  uint8_t Fcnt : 5;

  /*!
   * @brief
   * (FSPN + 1) is the count (span) of co-channels fired simultaneously in the
   * Firing Group.
   */
  uint8_t Fspn : 3;

  /*!
   * @brief Unsigned time fraction delay between co-channel firings.
   *
   * If FDLY is zero, all channels in the Firing Group were fired simultaneously
   * and the FSPN value may be ignored as there is no need to calculate a
   * per-channel time offset.
   *
   * If FDLY is non-zero, the channels were fired in co-channel groups separated
   * by FDLY. The span of each co-channel group is (FSPN+1). For a rolling
   * firing where each channel is fired separately, FSPN is 0. For a rolling
   * firing where two channels are fired together FSPN is 1. For a rolling
   * fireing where eight channels are fired together FSPN is 7.
   */
  uint8_t Fdly;

  //! @brief Horizontal direction , 0: Clockwise, 1: Counter-clockwise (1 bit)
  uint8_t Hdir : 1;

  //! @brief Vertical direction , 0: Upward, 1: Downward (1 bit)
  uint8_t Vdir : 1;

  //! @brief Vertical deflection angle (0.01 degree increments) [0..16383]
  uint16_t Vdfl : 14;

  //! @brief Azimuth (0.01 degree increments) [0..35999]
  uint16_t Azm;

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * - TOFFS is returned in nanoseconds.
   * - HDIR and VDIR are returned as their respective enums.
   * - VDFL and AZM are returned in degrees.
   */

  // Cast to 32-bit required to hold all values.
  uint32_t GetToffs const { return static_cast<uint32_t>(this->Toffs) * 64u; }
  uint8_t GetFcnt const { return this->Fcnt + 1; }
  uint8_t GetFspn const { return this->Fspn + 1; }
  GET_RAW(Fdly)
  GET_ENUM(HorizontalDirection, Hdir);
  GET_ENUM(VerticalDirection, Vdir);
  double GetVdfl const { return this->Vdfl * 0.01; }
  double GetAzm const { return this->Azm * 0.01; }
  //@}

  /*!
   * @brief Construct a FiringGroupHeader.
   * @param[in] data The packet data.
   * @param[in,out] i
   *   The offset to the start of the header. The offset will be advanced as the
   *   data is consumed.
   */
  FiringGroupHeader(PacketData & packetData)
  {
    packetData.SetFromBytes(this->Toffs);
    packetData.SetFromBits<1>(
      3, 5, this->Fcnt,
      0, 3, this->Fspn
    );
    packetData.SetFromByte(this->Fdly);
    packetData.SetFromBits<2>(
      15,  1, this->Hdir,
      14,  1, this->Vdir,
       0, 14, this->Vdfl
    );
    packetData.SetFromBytes(this->Azm);
  }
};

//------------------------------------------------------------------------------
// FiringHeader
//------------------------------------------------------------------------------
/*!
 * @brief Firing header of the VLP Advanced data packet format.
 */
class FiringHeader
{
private:
  //! @brief Logical channel number.
  uint8_t Lcn;

  //! @brief Firing mode.
  uint8_t Fm : 4;

  //! @brief Power level.
  uint8_t Pwr : 4;

  //! @brief Noise factor.
  uint8_t Nf;

  //! @brief Channel status flags.
  uint8_t Stat;

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * FM and STAT are returned as their respective enums.
   */
  GET_RAW(Lcn)
  GET_ENUM(FiringMode, Fm)
  GET_RAW(Pwr)
  GET_RAW(Nf)
  GET_ENUM(ChannelStatus, Stat)
  //@}

  /*!
   * @brief Construct a FiringGroupHeader.
   * @param[in] data The packet data.
   * @param[in,out] i
   *   The offset to the start of the header. The offset will be advanced as the
   *   data is consumed.
   */
  FiringHeader(PacketData & packetData)
  {
    packetData.SetFromByte(this->Lcn);
    packetData.SetFromBits<1>(
      4, 4, this->Fm,
      0, 4, this->Pwr
    );
    packetData.SetFromByte(this->Nf);
    packetData.SetFromByte(this->Stat);
  }


//------------------------------------------------------------------------------
// FiringReturn
//------------------------------------------------------------------------------
/*!
 * @brief Firing return of the VLP Advanced data packet format.
 *
 * The values in this struct may be encoded in the packet with 16 or 24 bits.
 * The size is determined at runtime by parsing the payload headers so a
 * template cannot be used here. The current implementation uses types large
 * enough to hold all possible values.
 *
 * @todo Consider using polymorphic types to avoid wasted space.
 */
// This class is not templated with "loadData" because it makes no sense to
// create an empty FiringReturn (it's basically just a wrapper around a vector
// with some bit-fiddling to access the return values).
class FiringReturn
{
private:
  //! @brief The packed data with the return values.
  std:vector<uint8_t> Data;

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Data);
  //@}

  //! @brief Reference to this return's firing.
  Firing const & MyFiring;

  //! @brief Templated subscript operator for accessing distance and intensities.
  template <typename T>
  T operator[](const int i)
  {
    // Range check.
    if (i < 0 || i >= this->MyFiring.MyFiringGroup.MyPayload.GetHeader().GetIntensityCount())
    {
      throw std::out_of_range("requested intensity is out of range of current set");
    }
    T value;
    auto index = i * this->BytesPerValue;
    setFromBytes(this->Data, index, value, this->BytesPerValue);
    return value;
  }

  //! @brief Get the distance from this firing.
  template <typename T>
  T GetDistance() { return this->operator[]<T>(0); }

  //! @brief Get an intensity from this firing by index.
  template <typename T>
  T GetIntensity(const int i) { return this->operator[]<T>(i+1); }

  //! @brief Get an intensity from this firing by type.
  template <typename T>
  T GetIntensity(IntensityType type) {
    // Check that the value is actually present otherwise the calculate index
    // will yield a different value or end up out of range.
    if (! (this->MyFiring.MyFiringGroup.MyPayload.GetHeader().GetIset() & type))
    {
      throw std::out_of_range("requested intensity type is not in the intensity set");
    }

    // Start at 1 to skip distance.
    int i = 1;

    // Count the number of bits set before the requested one. `mask` constains a
    // single set bit. By decrementing the value, we obtain a mask over all
    // preceding bits which we can then count to get the offset of the requested
    // intensity.
    uint8_t mask = static_cast<uint8_t>(type);
    if (mask)
    {
      mask--;
      i += SET_BITS_IN_BYTE[this->Iset & mask];
    }
    return this->operator[]<T>(i);
  }

  FiringReturn(Firing const & firing, PacketData & packetData)
    : FiringRef = firing
  {
    auto dataLength = this->MyFiring.MyFiringGroup.MyPayload.GetHeader().GetNumberOfBytesPerFiringReturn();
    packetData.CopyBytes(this->Data, dataLength);
  }
};

//------------------------------------------------------------------------------
// Firing
//------------------------------------------------------------------------------
/*!
 * @brief  Parse and optionally load data from a firing.
 * @tparam loadData If true, load firing data, otherwise skip it via
 *                  packetData.SkipBytes
 */
template <bool loadData>
class Firing
{
private:
  //! @brief Firing header.
  FiringHeader Header;

  //! @brief Returns in this firing.
  std::vector<FiringReturn<loadData>> Returns;

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Header);
  GET_CONST_REF(Returns);
  //@}

  //! @brief Reference to this firing's group.
  FiringGroup<loadData> const & MyFiringGroup;

  Firing(FiringGroup const & firingGroup, PacketData & packetData)
    : MyFiringGroup = firingGroup
  {
    this->Header = FiringHeader(packetData);


    if (loadData)
    {
      auto dcount = this->MyFiringGroup.MyPayload.GetHeader().GetDistanceCount();
      this->returns.reserve(dcount);
      for (decltype(dcount) j = 0; j < dcount; ++j)
      {
        FiringReturn<loadData> firingReturn = FiringReturn<loadData>((* this), packetData);
        this->returns.push_back(firingReturn);
      }
    }
    else
    {
      auto bytesPerFiring = this->MyFiringGroup.MyPayload.GetHeader().GetNumberOfBytesPerFiring();
      packetData.SkipBytes(bytesPerFiring);
    }
  }
}

//------------------------------------------------------------------------------
// FiringGroup
//------------------------------------------------------------------------------
/*!
 * @brief  Parse and optionally load data from a firing group.
 * @tparam loadData If true, load all firing data, otherwise only load firing
 *                  headers while skipping over return values.
 */
template <bool loadData>
class FiringGroup
{
private:
  //! @brief Firing group header.
  FiringGroupHeader Header;

  //! @brief Firings.
  std::vector<Firing<loadData>> Firings {0};

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Header);
  GET_CONST_REF(Firings);
  //@}

  //! @brief Reference to the payload containing this firing group.
  Payload<loadData> const & MyPayload;

  /*!
   * @param[in]     payload   The payload that contains this firing group.
   * @param[in]     data       Pointer to the packet bytes.
   * @param[in]     dataLength The byte length of the packet data.
   * @param[in,out] i          The offset to the data.
   */
  FiringGroup(Payload<loadData> const & payload, PacketData & packetData)
    : MyPayload = payload
  {
    // For word alignment.
    packetData.BeginBlock();

    // Group headers are padded to 32-bit boundaries. Use GLEN to advance the
    // index correctly.
    auto glen = this->MyPayload->GetGlen();
    packetData.BeginBlock();
    this->Header = FiringGroupHeader(packetData);
    packetData.EndBlock(this->MyPayload.GetGlen());

    auto fcnt = this->header.GetFcnt();
    this->Firings.reserve(fcnt);

    for (decltype(fcnt) j = 0; j < fcnt; ++j)
    {
      Firing<loadData> firing = Firing<loadData>((* this), packetData);
      this->Firings.push_back(firing);
    }

    // For word alignment.
    packetData.EndBlock();
  }
}

//------------------------------------------------------------------------------
// Payload
//------------------------------------------------------------------------------
/*!
 * @brief  Parse and optionally load data from the payload.
 * @tparam loadData If true, load all data, otherwise only load metadata (e.g.
 *                  firing group headers, extension headers).
 */

template <bool loadData>
class Payload
{
private:
  //! @brief Payload header.
  PayloadHeader Header;

  //! @brief Variable number of extension headers.
  std::vector<ExtensionHeader<loadData>> ExtensionHeaders {0};

  //! @brief Variable number of firing groups.
  std::vector<FiringGroup<loadData>> FiringGroups {0};

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Header);
  GET_CONST_REF(ExtensionHeaders);
  GET_CONST_REF(FiringGroups);
  //@}


  /*!
   * @brief Detect frame boundaries between firing groups.
   *
   * VeloView requires some concept of a frame. The current approach is to
   * detect the azimuth crossing zero but this will likely need to be changed in
   * the future.
   *
   * @param[out] newFrameBoundaries The indices of firing groups that start a
   *                                new frame will be pushed onto this vector.
   */
  void DetectFrames(FrameTracker & CurrentFrameTracker, std::vector<size_t> & newFrameBoundaries)
  {
    for (size_t i = 0; i < this->FiringGroups.size(); ++i)
    {
      FrameTracker frameTracker(this->FiringGroups[i]);
      if (this->CurrentFrameTracker.IsNewFrame(frameTracker))
      {
        newFrameBoundaries.push_back(i);
      }
      this->CurrentFrameTracker = frameTracker;
    }
  }

  Payload(PacketData & packetData)
  {
    this->header = PayloadHeader(packetData);

    // Check for extension headers.
    auto nxhdr = this->header.GetNxhdr();
    while (nxhdr != 0)
    {
      // The extension header automatically adjusts its length to end on a
      // 32-bit boundary so padding need not be handled here.
      ExtensionHeader<loadData> extensionHeader = ExtensionHeader<loadData>((* this), packetData);
      this->ExtensionHeaders.push_back(extensionHeader);
      nxhdr = extensionHeader.GetNxhdr();
    }

    // The rest of the data should be filled with firing groups.
    while (i < dataLength)
    {
      FiringGroup<loadData> firingGroup = FiringGroup<loadData>((* this), packetData);
      this->FiringGroups.push_back(firingGroup);
    }
  }
}

//------------------------------------------------------------------------------
// FrameTracker
//------------------------------------------------------------------------------
/*!
 * @brief Object to track information related to frame changes. All frame change
 *        logic should be kept here.
 * @todo  Develop this beyond simplying tracking rollovers of the azimuth.
 */
class FrameTracker
{
private:
  // HorizontalDirection Hdir = HD_UP;
  // VerticalDirection Vdir = VD_CLOCKWISE;
  // Set default values to something that will detect the first frame as a new
  // frame.
  // double Vdfl = -500.0;
  double  Azm = -500.0;

public:
  FrameTracker()
  {
  }
  FrameTracker(FiringGroup & firingGroup)
  {
    this->Azm = firingGroup.GetHeader().GetAzm();
  }

  /*!
   * @brief Returns true if the passed FrameTracker appears to be a new frame.
   * @todo  Consider better approaches to delimiting frames.
   */
  bool IsNewFrame(FrameTracker const & frameTracker)
  {
    return (std::abs(this->Azm - frameTracker) > 180.0)
  }
}


















//------------------------------------------------------------------------------
// VelodyneAdvancedPacketInterpreter methods.
//------------------------------------------------------------------------------
VelodyneAdvancedPacketInterpreter::VelodyneAdvancedPacketInterpreter()
{
}

//------------------------------------------------------------------------------
VelodyneAdvancedPacketInterpreter::~VelodyneAdvancedPacketInterpreter()
{
}

//------------------------------------------------------------------------------
VelodyneAdvancedPacketInterpreter::LoadCalibration(std::string const & filename)
{
  // TODO
}

//------------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::ProcessPacket(unsigned char const * data, unsigned int dataLength, int startPosition)
{
  // TODO: check how to handle startPosition
  // PacketData packetData = PacketData(data + startPosition, dataLength - startPosition, 0);
  PacketData packetData = PacketData(data, dataLength, 0);
  Payload payload;

  // The packet classes throw length errors if the packet does not contain the
  // expected length.
  try
  {
    payload = Payload<true>(packetData);
  }
  // Length errors are raised if the packet data does not conform to the
  // expected lengths. Returning here is basically the same thing as returning
  // at the start of this function if  IsLidarPacket() returns false. The
  // difference is that here the full data is checked instead of just the
  // header.
  catch (std::length_error const & e)
  {
    return;
  }


  PayloadHeader payloadHeader = payload.GetHeader();
  auto pseq = payloadHeader.GetPseq();
  auto iset = payloadHeader.GetIset();

  // TODO: make this configurable via the user interface
  uint8_t distanceIndex;
  DistanceType distanceType;
  if (payloadHeader.IsDsetMask())
  {
    auto dsetMask = payloadHeader.GetDsetMask();
    distanceType = firstSetBit(dsetMask);
    distanceIndex = indexOfBit(dsetMask, distanceType);
  }
  else
  {
    distanceIndex = 0;
    distanceType = DSET_FIRST;

  }
  auto distanceTypeString = toString(distanceType);

  for (auto firingGroup : payload.GetFiringGroups())
  {
    // Detect frame changes in firing groups.
    FrameTracker frameTracker = FrameTracker(firingGroup);
    if (this->CurrentFrameTracker.IsNewFrame(frameTracker))
    {
      this->SplitFrame();
    }
    this->CurrentFrameTracker = frameTracker;

    auto timeFractionOffset = firingGroup.GetToffs(); 
    auto coChannelTimeFractionDelay = firingGroup.GetFdly();
    double verticalAngle = firingGroup.GetVdfl();
    double azimuth = firingGroup.GetAzm();

    // TODO
    // This is just placeholder guesswork. Check how the data needs to be handled.
    double phi = degreesToRadians(verticalAngle);
    double theta = degreesToRadians(azimuth);
    double cosPhi = std::cos(phi);
    double sinPhi = std::sin(phi);
    double cosTheta = std::cos(theta);
    double sinTheta = std::sin(theta);

    for (auto firing : firingGroup.GetFirings())
    {
      auto channelNumber = firing.GetLCN();

      // The firing mode is an enum so we need to convert it to a human-readable
      // string.
      auto firingMode = firing.GetFM();
      auto firingModeString = ToString(firingMode);

      auto power = firing.GetPwr();
      auto noise = firing.GetNf();

      // Status is also an enum and requires a string conversion.
      auto status = firing.GetStat();
      auto statusString = ToString(status);


      // Only one distance type is displayed but there are multiple here.
      auto firingReturn = firing.GetReturns[distanceIndex];
      double distance = firingReturn.GetDistance();

      // TODO
      // No information has been received yet concerning how to actually
      // determine the distance from the returns.
      double x = distance * cosTheta * sinPhi;
      double y = distance * sinTheta * sinPhi;
      double z = distance * cosphi;

      // TODO
      // Determine which information is relevent and update accordingly.
      this->Points->InsertNextPoint({x,y,z});
      this->INFO_Xs->InsertNextValue(x);
      this->INFO_Ys->InsertNextValue(y);
      this->INFO_Zs->InsertNextValue(z);

      // TODO Replace these with the angles for the sensor after calibration.
      this->INFO_VerticalAngles->InsertNextValue(verticalAngle);
      this->INFO_Azimuths->InsertNextValue(azimuth);

      this->INFO_Pseqs->InsertNextValue(pseq);
      this->INFO_ChannelNumbers->InsertNextValue(channelNumber);
      this->INFO_FiringModeStrings->InsertNextValue(firingModeString);
      this->INFO_Powers->InsertNextValue(power);
      this->INFO_Noises->InsertNextValue(noise);
      this->INFO_StatusStrings->InsertNextValue(statusString);
      this->INFO_DistanceTypeStrings->InsertNextValue(distanceTypeString);

//! @brief Convenience macro for setting intensity values
#define INSERT_INTENSITY(my_array, iset_flag) \
      this->INFO_ ## my_array->InsertNextVaue((iset & (ISET_ ## iset_flag)) ? firingReturn.GetIntensity((ISET_ ## iset_flag)) : -1);

      // TODO: Make the inclusion of these columns fully optionally at runtime.

      // Add additional values here when ISET is expanded in future versions.
      INSERT_INTENSITY(Reflectivities, REFLECTIVITY)
      INSERT_INTENSITY(Intensities, INTENSITY)
      INSERT_INTENSITY(Confidences, CONFIDENCE)
    }
  }
}

//------------------------------------------------------------------------------
bool VelodyneAdvancedPacketInterpreter::IsLidarPacket(unsigned char const * data, unsigned int dataLength)
{
  // TODO
  // Determine the best way to check if a packet contains lidar data. Currently
  // this just checks that the packet data contains a plausible payload header.
  PacketData packetData = PacketData(data, dataLength, 0);
  try
  {
    PayloadHeader payloadHeader = PayloadHeader(packetData);
    return true;
  }
  catch (std::length_error const & e)
  {
    return false;
  }
  //
}

//----------------------------------------------------------------------------
/*!
 * @brief         Initialize an array.
 * @tparam        T                The type of the array. This is templated so
 *                                 that the caller does not need to consider the
 *                                 type, which may change with the
 *                                 specification.
 * @param[in,out] array            The input array.
 * @param[in]     numberOfElements The number of elements that the array must be
 *                                 able to hold after initialization.
 */
template <typename T>
inline
void InitializeDataArray(
  T & array,
  char const * name,
  vtkIdType numberOfElements
)
{
  array = T::New();
  array->Allocate(numberOfElements);
  array->SetName(name);
}

//----------------------------------------------------------------------------
/*!
 * @brief         Initialize an array for datapoint attributes and add it to the
 *                polydata.
 * @tparam        T                The type of the array. This is templated so
 *                                 that the caller does not need to consider the
 *                                 type, which may change with the
 *                                 specification.
 * @param[in,out] array            The input array.
 * @param[in]     numberOfElements The number of elements that the array must be
 *                                 able to hold after initialization.
 * @param[out]    polyData         The polydata instance to which the array
 *                                 should be added.
 * 
 * This is just a convencience wrapper around InitializeDataArray.
 */
template <typename T>
inline
void InitializeDataArrayForPolyData(
  T & array,
  char const * name,
  vtkIdType numberOfElements,
  vtkPolyData * polyData
)
{
  InitializeDataArray(array, name, numberOfElements);
  if (polyData)
  {
    polyData->GetPointData()->AddArray(array);
  }
}

//------------------------------------------------------------------------------
vtkSmartPointer<vtkPolyData> VelodyneAdvancedPacketInterpreter::CreateNewEmptyFrame(vtkIdType numberOfPoints)
{
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();

  // Points.
  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  // The frame size must be large enough to contain the requested number of
  // points.
  this->FrameSize = std::max(
    this->FrameSize,
    static_cast<decltype(this->FrameSize)>(numberOfPoints)
  );
  if (this->FrameSize > 0)
  {
    points->Allocate(this->FrameSize);
    points->SetNumberOfPoints(numberOfPoints);
  }

  // Same name as vtkVelodyneHDLReader.
  // TODO
  // Define the name as a static constant member of the parent class if it
  // should be common to all subclasses.
  points->GetData()->SetName("Points_m_XYZ");

  // Point the polydata to the points.
  polyData->SetPoints(points.GetPointer());

  // Replace the old points.
  this->Points = points.GetPointer();

  // Replace and initialize all of the associated data arrays.
  //
  // Use a template here to make this section of code type-agnostic. The types
  // of the different datapoint attributes may change in the future with
  // evolving packet specifications.

// Convencience macro
#define INIT_INFO_ARR(arr_name, disp_name) \
  InitializeDataArrayForPolyData(this->INFO_ ## arr_name, disp_name, numberOfPoints, polyData);

  INIT_INFO_ARR(Xs                  , "X")
  INIT_INFO_ARR(Ys                  , "Y")
  INIT_INFO_ARR(Zs                  , "Z")
  INIT_INFO_ARR(Azimuths            , "Azimuth")
  INIT_INFO_ARR(VerticalAngles      , "Vertical Angle")
  INIT_INFO_ARR(DistanceTypeStrings , "Distance Type")
  INIT_INFO_ARR(FiringModeStrings   , "FiringMode")
  INIT_INFO_ARR(StatusStrings       , "Status")
  INIT_INFO_ARR(Intensities         , "Intensity")
  INIT_INFO_ARR(Confidences         , "Confidence")
  INIT_INFO_ARR(Reflectivities      , "Reflectivity")
  INIT_INFO_ARR(ChannelNumbers      , "Logical Channel Number")
  INIT_INFO_ARR(Powers              , "Power")
  INIT_INFO_ARR(Noises              , "Noise")
  INIT_INFO_ARR(Pseqs               , "Packet Sequence Number")

  return polyData;
}

//------------------------------------------------------------------------------
// TODO: Revisit this if the frequency still needs to be calculated here.
bool VelodyneAdvancedPacketInterpreter::SplitFrame(bool force)
{
  this->FrameSize = std::max(
    this->FrameSize,
    static_cast<decltype(this->FrameSize)>(this->Points->size())
  );
  return this->LidarPacketInterpretor::SplitFrame(force);
}

//------------------------------------------------------------------------------
// void VelodyneAdvancedPacketInterpreter::ResetCurrentFrame()
// {
// }

//------------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::PreProcessPacket(unsigned char const * data, unsigned int dataLength, bool & isNewFrame, int & framePositionInPacket)
{
  PacketData packetData = PacketData(data, dataLength, 0);
  PayloadHeader packetHeader = PayloadHeader<false>(packetData);
  std::vector<size_t> newFrameBoundaries {0};

  // TODO
  // Review how this is supposed to work. A packet contains a single payload so
  // it's not possible to jump directly to a new frame, and several frames could
  // theoretically be included in a single payload depending on how frames are
  // determined. If the function limits this to one then perhaps we are losing
  // frame data here.
  packetHeader.DetectFrames(this->CurrentFrameTracker, newFrameBoundaries);
  isNewFrame = (newFrameBoundaries.size() > 0);
  framePositionInPacket = 0;
}


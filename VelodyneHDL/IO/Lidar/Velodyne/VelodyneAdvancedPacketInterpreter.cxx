#include "VelodyneAdvancedPacketInterpreter.h"

#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

#include "vtkDataPacket.h"
using namespace DataPacketFixedLength;

#include <type_traits>
#include <boost/preprocessor.hpp>

#include <cstring>

#include <iostream>
#define DEBUG_MSG(msg) std::cout << "DEBUG:" << msg << " [" << __LINE__ << "] " << std::endl;

//------------------------------------------------------------------------------
// General macros constants.
//------------------------------------------------------------------------------
//! @brief Lengths in the headers are given in units of 32-bit words.
#define BYTES_PER_HEADER_WORD 4u

//------------------------------------------------------------------------------
// Accessor macros.
//------------------------------------------------------------------------------
//! @brief Simple getter for uninterpretted values.
#define GET_RAW(attr) decltype(attr) Get ## attr () const { return this->attr; }

//! @brief Get a const reference.
#define GET_CONST_REF(attr) decltype(attr) const & Get ## attr () const { return this->attr; }

//! @brief Getter for enum values that are stored as raw values internally.
#define GET_ENUM(enumtype, attr) enumtype Get ## attr () const { return to ## enumtype (this->attr); }

//! @brief Get a header length in bytes.
#define GET_LENGTH(attr) size_t Get ## attr () const { return (this->attr * BYTES_PER_HEADER_WORD); }

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

//! @brief Model identification code.
DEFINE_ENUM(
  ModelIdentificationCode,
  MIC_,
  ((RESERVED , 0))
  ((VLP16    , 1))
  ((VLP16_HD , 2))
  ((VLP32A   , 3))
  ((VLP32B   , 4)),
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
template <typename T, typename S>
T indexOfBit(T value, S bit)
{
  T castBit = static_cast<T>(bit);
  T precedingBits = (castBit - 1) & value;
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
  size_t numberOfBytes = sizeof(ValueT)
)
{
  IndexT stop = index + numberOfBytes;
  value = bytes[index++];
  while (index < stop)
  {
    value =  (value * (static_cast<ValueT>(1) << 8)) + bytes[index++];
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
  TS const mask = (static_cast<TS>(1) << number) - 1;
  destination = (source >> offset) & mask;
}

/*!
 * @copydoc setFromBits
 * @brief   Variadic overload to set multiple destination values from the same
 *          bytes. Extra values are passed as triples of offset, number and
 *          destination.
 * @todo    Check if this has any noticeable performance overhead.
 */
// template <typename TS, typename TD, typename ... RemainingArgs>
// inline
// void setFromBits(TS & source, uint8_t offset, uint8_t number, TD & destination, RemainingArgs ... remainingArgs)
// {
//   setFromBits(source, offset, number, destination);
//   setFromBits(source, remainingArgs...);
// }

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
//template <size_t WordSize = BYTES_PER_HEADER_WORD>
template <size_t WordSize>
class PacketDataHandle
{
private:
  //! @brief The raw data.
  uint8_t const * Data;

  //! @brief The number of bytes in the raw data.
  size_t Length;

  //! @brief The index of the next byte to process.
  size_t Index;

  //! @brief Tracked data for aligning blocks on word limits.
  std::vector<size_t> BlockStartIndices;

public:
  //@{
  //! @brief Getters.
  GET_RAW(Data);
  GET_RAW(Length);
  GET_RAW(Index);
  //@}

  PacketDataHandle(decltype(Data) data, decltype(Length) length, decltype(Index) index)
    : Data {data}, Length {length}, Index {index}
  {
  }

  /*!
   * @brief Mark the beginning of a block to enabled automatic word alignment
   *        when it ends.
   * @param[in] wordSize The word size on which to align the block.
   *
   * This should be called before any values are read from the block. When all
   * values have been read, call EndBlock().
   */
  void BeginBlock()
  {
    this->BlockStartIndices.push_back(this->Index);
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
    size_t blockStartIndex = this->BlockStartIndices.back();
    this->BlockStartIndices.pop_back();
    auto wordRemainder = (this->Index - blockStartIndex) % WordSize;
    if (wordRemainder > 0)
    {
      this->Index += (WordSize - wordRemainder);
    }
  }
  void EndBlock(size_t length)
  {
    size_t blockStartIndex = this->BlockStartIndices.back();
    this->BlockStartIndices.pop_back();
    if ((this->Index - blockStartIndex) > length)
    {
      throw std::length_error("EndBlock(): block size length is less than the number of consumed bytes since BeginBlock() was called");
    }
    this->Index = blockStartIndex + length;
  }
  //@}
  
  /*!
   * @brief Reset the index to the beginning of the current block and remove it.
   */
  void ResetBlock()
  {
    size_t blockStartIndex = this->BlockStartIndices.back();
    this->BlockStartIndices.pop_back();
    this->Index = blockStartIndex;
  }
  
  //! @brief Get the number of bytes remaining from the current index to the end
  //         of the data as defined by the length.
  size_t GetRemainingLength()
  {
    return (this->Length > this->Index) ? (this->Length - this->Index) : 0;
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
   *                         This must be at least 1. Values larger than the
   *                         size of uint86_t are not supported.
   * @tparam PassthroughArgs All arguments to pass through to SetFromBits.
   * 
   * This is a wrapper around SetFromBits with automatic deduction of the
   * smallest type required to hold the number of bytes to process.
   */
	// TODO: add insert to fail if numberOfBytes is greater than 8
  // template <uint8_t numberOfBytes, typename ... PassthroughArgs>
  // void SetFromBits(PassthroughArgs ... passthroughArgs)
  template <uint8_t numberOfBytes, typename TD1, typename TD2>
  void SetFromBits(
    uint8_t offset1, uint8_t number1, TD1 & destination1,
    uint8_t offset2, uint8_t number2, TD2 & destination2
  )
  {
    // Select a placeholder type large enough to hold the required value. All
    // bits are counted from the least significant bit so bits in the padding
    // byte(s), if any, will be ignored.
    static_assert(numberOfBytes > 0 && numberOfBytes <= sizeof(uint64_t), "SetFromBits: numberOfBytes template parameter must be at least 1 and at most sizeof(uint64_t).");
    typename std::conditional<
      (numberOfBytes == 1),
      uint8_t,
      typename std::conditional<
        (numberOfBytes == 2),
        uint16_t,
        typename std::conditional<(numberOfBytes <= 4), uint32_t, uint64_t>::type
      >::type
    >::type placeholder;

    this->SetFromBytes(placeholder, numberOfBytes);
    // setFromBits(placeholder, passthroughArgs...);
    setFromBits(placeholder, offset1, number1, destination1);
    setFromBits(placeholder, offset2, number2, destination2);
  }
  template <uint8_t numberOfBytes, typename TD1, typename TD2, typename TD3>
  void SetFromBits(
    uint8_t offset1, uint8_t number1, TD1 & destination1,
    uint8_t offset2, uint8_t number2, TD2 & destination2,
    uint8_t offset3, uint8_t number3, TD3 & destination3
  )
  {
    // Select a placeholder type large enough to hold the required value. All
    // bits are counted from the least significant bit so bits in the padding
    // byte(s), if any, will be ignored.
    static_assert(numberOfBytes > 0 && numberOfBytes <= sizeof(uint64_t), "SetFromBits: numberOfBytes template parameter must be at least 1 and at most sizeof(uint64_t).");
    typename std::conditional<
      (numberOfBytes == 1),
      uint8_t,
      typename std::conditional<
        (numberOfBytes == 2),
        uint16_t,
        typename std::conditional<(numberOfBytes <= 4), uint32_t, uint64_t>::type
      >::type
    >::type placeholder;

    this->SetFromBytes(placeholder, numberOfBytes);
    // setFromBits(placeholder, passthroughArgs...);
    setFromBits(placeholder, offset1, number1, destination1);
    setFromBits(placeholder, offset2, number2, destination2);
    setFromBits(placeholder, offset3, number3, destination3);
  }

  /*!
   * @brief      Copy a sequence of bytes (wrapper around std::memcpy)..
   * @param[out] data   The destination to which to copy the data.
   * @param[in]  length The number of bytes to copy.
   */
  void CopyBytes(std::vector<uint8_t> & data, size_t length)
  {
    data.reserve(length);
    std::memcpy(data.data(), this->Data + this->Index, length);
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


  // TODO: remove
  void PrintBytes(size_t length)
  {
    std::cout << std::hex;
    for (size_t i = 0; i < length; ++i)
    {
      std::cout << +(this->Data[this->Index + i]) << " ";
    }
    std::cout << std::dec << std::endl;
  }
};

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
  uint8_t Ver ; //: 4;

  //! @brief Header length (min: 6)
  uint8_t Hlen ; //: 4;

  //! @brief Next header type.
  uint8_t Nxhdr;

  //! @brief Firing group header length.
  uint8_t Glen ; //: 4;

  //! @brief Firing header length.
  uint8_t Flen ; //: 4;

  //! @brief Model Identification Code
  uint8_t Mic;

  //! @brief Time status (TBD).
  uint8_t Tstat;

  //! @brief Distance set.
  uint8_t Dset;

  //! @brief Intensity set.
  uint16_t Iset;

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
  GET_RAW(Pseq)
  //@}


  //! @brief The number of bytes per value in a return.
  uint8_t GetDistanceSizeInBytes() const
  {
    return ((this->Dset & (1 << 7)) ? 3 : 2);
  }

  //! @brief True if the distance set includes a mask, false if a count.
  bool IsDsetMask() const
  {
    return ! (this->Dset & (1 << 6));
  }

  //! @brief The mask or count, depending on the return value of isDsetMask.
  uint8_t GetDsetMask() const
  {
    return (this->Dset & ((1 << 6) - 1));
  }

  //! @brief Get the number of distances in each firing group.
  uint8_t GetDistanceCount() const
  {
    // The "mask" is actually a count if IsDsetMask is false.
    auto mask = this->GetDsetMask();
    return (this->IsDsetMask()) ? SET_BITS_IN_BYTE[mask] : mask;

  }

  //! @brief Get the number of intensities in each firing.
  uint8_t GetIntensityCount() const
  {
    return SET_BITS_IN_BYTE[this->Iset];
  }

  //! @brief The the number of bytes per firing return.
  size_t GetNumberOfBytesPerFiringReturn() const
  {
    size_t bytesPerDistance = this->GetDistanceSizeInBytes();
    size_t icount = this->GetIntensityCount();
    return bytesPerDistance + icount;
  }

  //! @brief The the number of bytes per firing.
  size_t GetNumberOfBytesPerFiring() const
  {
    size_t dcount = this->GetDistanceCount();
    size_t bytesPerReturn = this->GetNumberOfBytesPerFiringReturn();
    return dcount * bytesPerReturn;
  }

  /*!
   * @brief         Construct a PayloadHeader.
   * @param[in,out] packetDataHandle The packet data from which to parse the header.
   */
  PayloadHeader(PacketDataHandle<BYTES_PER_HEADER_WORD> & packetDataHandle)
  {
    // For word alignment.
    packetDataHandle.BeginBlock();
    packetDataHandle.SetFromBits<1>(
      4, 4, this->Ver,
      0, 4, this->Hlen
    );

    // Use GetHlen() to get the count in bytes.
    if (this->GetHlen() > packetDataHandle.GetRemainingLength())
    {
      packetDataHandle.ResetBlock();
      throw std::length_error("data does not contain enough bytes for payload header");
    }

    packetDataHandle.SetFromByte(this->Nxhdr);
    packetDataHandle.SetFromBits<1>(
      4, 4, this->Glen,
      0, 4, this->Flen
    );
    packetDataHandle.SetFromByte(this->Mic);
    packetDataHandle.SetFromByte(this->Tstat);
    packetDataHandle.SetFromByte(this->Dset);
    packetDataHandle.SetFromBytes(this->Iset);
    packetDataHandle.SetFromBytes(this->Tref);
    packetDataHandle.SetFromBytes(this->Pseq);
    // For word alignment.
    packetDataHandle.EndBlock(this->GetHlen());
  }
};


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
  uint8_t Hlen;

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
  ExtensionHeader(PacketDataHandle<BYTES_PER_HEADER_WORD> & packetDataHandle)
  {
    packetDataHandle.SetFromByte(this->Hlen);

    auto hlen = this->GetHlen();
    if (hlen > packetDataHandle.GetRemainingLength())
    {
      throw std::length_error("data does not contain enough bytes for extension header");
    }

    packetDataHandle.SetFromByte(this->Nxhdr);

    auto dataLen = hlen - (sizeof(this->Hlen) + sizeof(this->Nxhdr));

    // Only copy the data if the template parameter requests it.
    if (loadData)
    {
      packetDataHandle.CopyBytes(this->Data, dataLen);
    }
    else
    {
      packetDataHandle.SkipBytes(dataLen);
    }
  }
};

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
  uint8_t Fcnt ; //: 5;

  /*!
   * @brief
   * (FSPN + 1) is the count (span) of co-channels fired simultaneously in the
   * Firing Group.
   */
  uint8_t Fspn ; //: 3;

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
  uint8_t Hdir ; //: 1;

  //! @brief Vertical direction , 0: Upward, 1: Downward (1 bit)
  uint8_t Vdir ; //: 1;

  //! @brief Vertical deflection angle (0.01 degree increments) [0..16383]
  uint16_t Vdfl ; //: 14;

  //! @brief Azimuth (0.01 degree increments) [0..35999]
  uint16_t Azm;

public:
  //@{
  /*!
   * @brief Getters for header values.
   *
   * - TOFFS is returned in nanoseconds.
   * - HDIR and VDIR are returned as their respective enums.
   * - VDFL and AZM are returned as an integral number of hundredths of degrees.
   */

  // Cast to 32-bit required to hold all values.
  uint32_t GetToffs() const { return static_cast<uint32_t>(this->Toffs) * 64u; }
  uint8_t GetFcnt() const { return this->Fcnt + 1; }
  uint8_t GetFspn() const { return this->Fspn + 1; }
  GET_RAW(Fdly)
  GET_ENUM(HorizontalDirection, Hdir)
  GET_ENUM(VerticalDirection, Vdir)
  GET_RAW(Vdfl)
  GET_RAW(Azm)
  //@}

  //@{
  //! @brief Convenience accessors to get angles in degrees.
  double GetVerticalDeflection() const { return this->Vdfl * 0.01; }
  double GetAzimuth() const { return this->Azm * 0.01; }
  //@}

  /*!
   * @brief         Construct a FiringGroupHeader.
   * @param[in]     data The packet data.
   * @param[in,out] i    The offset to the start of the header. The offset will
   *                     be advanced as the data is consumed.
   */
  FiringGroupHeader(PacketDataHandle<BYTES_PER_HEADER_WORD> & packetDataHandle)
  {
    packetDataHandle.SetFromBytes(this->Toffs);
    packetDataHandle.SetFromBits<1>(
      3, 5, this->Fcnt,
      0, 3, this->Fspn
    );
    packetDataHandle.SetFromByte(this->Fdly);
    packetDataHandle.SetFromBits<2>(
      15,  1, this->Hdir,
      14,  1, this->Vdir,
       0, 14, this->Vdfl
    );
    packetDataHandle.SetFromBytes(this->Azm);
  }

  // A default constructor is required when member initialization is not
  // possible.
  FiringGroupHeader()
  {
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
  uint8_t Fm ; //: 4;

  //! @brief Power level.
  uint8_t Pwr ; //: 4;

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
   * @brief         Construct a FiringGroupHeader.
   * @param[in]     data The packet data.
   * @param[in,out] i    The offset to the start of the header. The offset will
   *                     be advanced as the data is consumed.
   */
  FiringHeader(PacketDataHandle<BYTES_PER_HEADER_WORD> & packetDataHandle)
  {
    packetDataHandle.SetFromByte(this->Lcn);
    packetDataHandle.SetFromBits<1>(
      4, 4, this->Fm,
      0, 4, this->Pwr
    );
    packetDataHandle.SetFromByte(this->Nf);
    packetDataHandle.SetFromByte(this->Stat);
  }
};


// Forward declarations
template <bool loadData>
class Firing;

template <bool loadData>
class FiringGroup;

template <bool loadData>
class Payload;

//------------------------------------------------------------------------------
// FiringReturn
//------------------------------------------------------------------------------
/*!
 * @brief  Firing return of the VLP Advanced data packet format.
 * @tparam loadData If true, load firing data, otherwise skip it via
 *                  packetDataHandle.SkipBytes
 * @todo   Consider using polymorphic types to avoid wasted space.
 * 
 * The values in this struct may be encoded in the packet with 16 or 24 bits.
 * The size is determined at runtime by parsing the payload headers so a
 * template cannot be used here. The current implementation uses types large
 * enough to hold all possible values.
 */
// This class is not templated with "loadData" because it makes no sense to
// create an empty FiringReturn (it's basically just a wrapper around a vector
// with some bit-fiddling to access the return values).
template <bool loadData>
class FiringReturn
{
private:
  //! @brief The packed data with the return values.
  std::vector<uint8_t> Data;

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Data);
  //@}

  //! @brief Reference to this return's firing.
  Firing<loadData> const * FiringPtr;

  //! @brief Templated subscript operator for accessing distance and intensities.
  template <typename T>
  T operator[](const int i) const
  {
    PayloadHeader const & payloadHeader = this->FiringPtr->FiringGroupPtr->PayloadPtr->GetHeader();
    // Range check.
    if (i < 0 || i >= payloadHeader.GetIntensityCount())
    {
      throw std::out_of_range("requested intensity is out of range of current set");
    }
    auto bytesPerDistance = payloadHeader.GetDistanceSizeInBytes();
    if (i == 0)
    {
      T value;
      auto index = i;
      setFromBytes(this->Data.data(), index, value, bytesPerDistance);
      return value;
    }
    else
    {
      return this->Data[bytesPerDistance + (i - 1)];
    }
  }

  //! @brief Get the distance from this firing.
  template <typename T>
  T GetDistance() const { return this->operator[]<T>(0); }

  // Default return type is 32-bit because the type may be either 16 or 24 bit.

  //! @brief Get an intensity from this firing by index.
  template <typename T>
  T GetIntensity(const int i) const { return this->operator[]<T>(i+1); }

  //! @brief Get an intensity from this firing by type.
  template <typename T>
  T GetIntensity(IntensityType type) const
  {
    // Check that the value is actually present otherwise the calculate index
    // will yield a different value or end up out of range.
    if (! (this->FiringPtr->FiringGroupPtr->PayloadPtr->GetHeader().GetIset() & type))
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
      PayloadHeader const & payloadHeader = this->FiringPtr->FiringGroupPtr->PayloadPtr->GetHeader();
      i += SET_BITS_IN_BYTE[payloadHeader.GetIset() & mask];
    }
    return this->operator[]<T>(i);
  }

  FiringReturn(Firing<loadData> const & firing, PacketDataHandle<BYTES_PER_HEADER_WORD> & packetDataHandle)
    : FiringPtr (& firing)
  {
    auto dataLength = this->FiringPtr->FiringGroupPtr->PayloadPtr->GetHeader().GetNumberOfBytesPerFiringReturn();
    packetDataHandle.CopyBytes(this->Data, dataLength);
  }

  FiringReturn & operator=(FiringReturn const & other)
  {
    this->FiringPtr = other.FiringPtr;
    this->Data = other.Data;
  }
};

//------------------------------------------------------------------------------
// Firing
//------------------------------------------------------------------------------
/*!
 * @brief  Parse and optionally load data from a firing.
 * @tparam loadData If true, load firing data, otherwise skip it via
 *                  packetDataHandle.SkipBytes
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
  FiringGroup<loadData> const * FiringGroupPtr;

  Firing(FiringGroup<loadData> const & firingGroup, PacketDataHandle<BYTES_PER_HEADER_WORD> & packetDataHandle)
    : Header (packetDataHandle)
    , FiringGroupPtr (& firingGroup)
  {
    if (loadData)
    {
      auto dcount = this->FiringGroupPtr->PayloadPtr->GetHeader().GetDistanceCount();
      this->Returns.reserve(dcount);
      for (decltype(dcount) j = 0; j < dcount; ++j)
      {
        this->Returns.push_back(FiringReturn<loadData>((* this), packetDataHandle));
      }
    }
    else
    {
      auto bytesPerFiring = this->FiringGroupPtr->PayloadPtr->GetHeader().GetNumberOfBytesPerFiring();
      packetDataHandle.SkipBytes(bytesPerFiring);
    }
  }

  Firing & operator=(Firing const & other)
  {
    this->FiringGroupPtr = other.FiringGroupPtr;
    this->Header = other.Header;
    this->Returns = other.Returns;
    for (auto retrn : this->Returns)
    {
      retrn.FiringPtr = this;
    }
  }
};

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
  std::vector<Firing<loadData>> Firings;

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Header);
  GET_CONST_REF(Firings);
  //@}

  //! @brief Reference to the payload containing this firing group.
  Payload<loadData> const * PayloadPtr;

  /*!
   * @param[in]     payload    The payload that contains this firing group.
   * @param[in]     data       Pointer to the packet bytes.
   * @param[in]     dataLength The byte length of the packet data.
   * @param[in,out] i          The offset to the data.
   */
  FiringGroup(Payload<loadData> const & payload, PacketDataHandle<BYTES_PER_HEADER_WORD> & packetDataHandle)
    : PayloadPtr (& payload)
  {
    // For word alignment.
    packetDataHandle.BeginBlock();

    // Group headers are padded to 32-bit boundaries. Use GLEN to advance the
    // index correctly.
    packetDataHandle.BeginBlock();
    this->Header = FiringGroupHeader(packetDataHandle);
    packetDataHandle.EndBlock(this->PayloadPtr->GetHeader().GetGlen());

    auto fcnt = this->Header.GetFcnt();
    this->Firings.reserve(fcnt);

    for (decltype(fcnt) j = 0; j < fcnt; ++j)
    {
      Firing<loadData> firing = Firing<loadData>((* this), packetDataHandle);
      this->Firings.push_back(firing);
    }

    // For word alignment.
    packetDataHandle.EndBlock();
  }

  FiringGroup & operator=(FiringGroup const & other)
  {
    this->PayloadPtr = other.PayloadPtr;
    this->Header = other.Header;
    // this->Firings = other.Firings;
    for (auto firing : other.Firings)
    {
      this->Firings.push_back(firing);
      this->Firings.back().FiringGroupPtr = this;
    }
  }
};

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

  template <bool loadData>
  FrameTracker(FiringGroup<loadData> & firingGroup)
  {
    this->Azm = firingGroup.GetHeader().GetAzm();
  }

  FrameTracker & operator=(FrameTracker const & other)
  {
    this->Azm = other.Azm;
    return (* this);
  }

  /*!
   * @brief Returns true if the passed FrameTracker appears to be a new frame.
   * @todo  Consider better approaches to delimiting frames.
   */
  bool IsNewFrame(FrameTracker const & frameTracker)
  {
    return (std::abs(this->Azm - frameTracker.Azm) > 180.0);
  }
};

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
  std::vector<ExtensionHeader<loadData>> ExtensionHeaders;

  //! @brief Variable number of firing groups.
  std::vector<FiringGroup<loadData>> FiringGroups;

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
  void DetectFrames(FrameTracker & currentFrameTracker, std::vector<size_t> & newFrameBoundaries)
  {
    for (size_t i = 0; i < this->FiringGroups.size(); ++i)
    {
      FrameTracker frameTracker = FrameTracker(this->FiringGroups[i]);
      if (currentFrameTracker.IsNewFrame(frameTracker))
      {
        newFrameBoundaries.push_back(i);
      }
      currentFrameTracker = frameTracker;
    }
  }

  Payload(PacketDataHandle<BYTES_PER_HEADER_WORD> & packetDataHandle)
    : Header {packetDataHandle}
  {
    // Check for extension headers.
    auto nxhdr = this->Header.GetNxhdr();
    while (nxhdr != 0)
    {
      // The extension header automatically adjusts its length to end on a
      // 32-bit boundary so padding need not be handled here.
      ExtensionHeader<loadData> extensionHeader = ExtensionHeader<loadData>(packetDataHandle);
      this->ExtensionHeaders.push_back(extensionHeader);
      nxhdr = extensionHeader.GetNxhdr();
    }

    // The rest of the data should be filled with firing groups.
    while (packetDataHandle.GetRemainingLength() > 0)
    {
      FiringGroup<loadData> firingGroup((* this), packetDataHandle);
      this->FiringGroups.push_back(firingGroup);
    }
  }
};

















//------------------------------------------------------------------------------
// VelodyneAdvancedPacketInterpreter methods.
//------------------------------------------------------------------------------
VelodyneAdvancedPacketInterpreter::VelodyneAdvancedPacketInterpreter()
{
	this->Init();
  this->DistanceResolutionM = 0.002;
  this->CurrentFrameTracker = new FrameTracker();
}

//------------------------------------------------------------------------------
VelodyneAdvancedPacketInterpreter::~VelodyneAdvancedPacketInterpreter()
{
  delete this->CurrentFrameTracker;
}

//------------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::ProcessPacket(unsigned char const * data, unsigned int dataLength, int startPosition)
{
  // TODO: check how to handle startPosition
  // PacketDataHandle packetDataHandle = PacketDataHandle(data + startPosition, dataLength - startPosition, 0);
  PacketDataHandle<BYTES_PER_HEADER_WORD> packetDataHandle = PacketDataHandle<BYTES_PER_HEADER_WORD>(data, dataLength, 0);

  Payload<true> payload {packetDataHandle};
/*
  // The packet classes throw length errors if the packet does not contain the
  // expected length.
  try
  {
    payload = Payload<true>(packetDataHandle);
  }
  // Length errors are thrown if the packet data does not conform to the
  // expected lengths. Returning here is basically the same thing as returning
  // at the start of this function if  IsLidarPacket() returns false. The
  // difference is that here the full data is checked instead of just the
  // header.
  catch (std::length_error const & e)
  {
    return;
  }
*/


  PayloadHeader payloadHeader = payload.GetHeader();
  auto pseq = payloadHeader.GetPseq();
  auto iset = payloadHeader.GetIset();

  // 64-bit PTP truncated format.
  auto timeRef = payloadHeader.GetTref();

  // TODO: make this configurable via the user interface
  uint8_t distanceIndex;
  DistanceType distanceType;
  if (payloadHeader.IsDsetMask())
  {
    auto dsetMask = payloadHeader.GetDsetMask();
    distanceType = toDistanceType(firstSetBit(dsetMask));
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
    if (this->CurrentFrameTracker->IsNewFrame(frameTracker))
    {
      this->SplitFrame();
    }
    (* (this->CurrentFrameTracker)) = frameTracker;

    auto firingGroupHeader = firingGroup.GetHeader();
    auto timeFractionOffset = firingGroupHeader.GetToffs(); 
    auto coChannelSpan = firingGroupHeader.GetFspn();
    auto coChannelTimeFractionDelay = firingGroupHeader.GetFdly();
    // double verticalAngle = firingGroup.GetVdfl();
    auto azimuth = firingGroupHeader.GetAzm();

    auto firings = firingGroup.GetFirings();
    size_t numberOfFirings = firings.size();

    for (size_t i = 0; i < numberOfFirings; ++i)
    {
      // TODO
      // This assumes that the spans are returned in order in the firing group.
      // Check that this is the case. If not, determine how to handle this (by
      // using the channe number?).
      uint32_t channelTimeFractionOffset = timeFractionOffset + (coChannelTimeFractionDelay * (i / coChannelSpan));

      auto firing = firings[i];
      auto firingHeader = firing.GetHeader();
      auto channelNumber = firingHeader.GetLcn();

      // The firing mode is an enum so we need to convert it to a human-readable
      // string.
      auto firingMode = firingHeader.GetFm();
      auto firingModeString = toString(firingMode);

      auto power = firingHeader.GetPwr();
      auto noise = firingHeader.GetNf();

      // Status is also an enum and requires a string conversion.
      auto status = firingHeader.GetStat();
      auto statusString = toString(status);


      // Only one distance type is displayed but there may be multiple in the
      // packet.
      auto firingReturn = firing.GetReturns()[distanceIndex];
      double distance;

      double position[3];
      this->ComputeCorrectedValues(
          azimuth, 
          firingReturn,
          i,
          position
      );

      // TODO
      // Determine which information is relevent and update accordingly.
      this->Points->InsertNextPoint(position);
      this->INFO_Xs->InsertNextValue(position[0]);
      this->INFO_Ys->InsertNextValue(position[1]);
      this->INFO_Zs->InsertNextValue(position[2]);

      // TODO Replace these with the angles for the sensor after calibration.
      // this->INFO_VerticalAngles->InsertNextValue(verticalAngle);
      this->INFO_Azimuths->InsertNextValue(azimuth);

      this->INFO_Pseqs->InsertNextValue(pseq);
      this->INFO_ChannelNumbers->InsertNextValue(channelNumber);
      this->INFO_TimeFractionOffsets->InsertNextValue(channelTimeFractionOffset);
      this->INFO_FiringModeStrings->InsertNextValue(firingModeString);
      this->INFO_Powers->InsertNextValue(power);
      this->INFO_Noises->InsertNextValue(noise);
      this->INFO_StatusStrings->InsertNextValue(statusString);
      this->INFO_DistanceTypeStrings->InsertNextValue(distanceTypeString);

//! @brief Convenience macro for setting intensity values
#define INSERT_INTENSITY(my_array, iset_flag) \
      this->INFO_ ## my_array->InsertNextValue((iset & (ISET_ ## iset_flag)) ? firingReturn.GetIntensity<uint32_t>((ISET_ ## iset_flag)) : -1);

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
  PacketDataHandle<BYTES_PER_HEADER_WORD> packetDataHandle = PacketDataHandle<BYTES_PER_HEADER_WORD>(data, dataLength, 0);
  try
  {
    PayloadHeader payloadHeader = PayloadHeader(packetDataHandle);
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
vtkSmartPointer<vtkPolyData> VelodyneAdvancedPacketInterpreter::CreateNewEmptyFrame(vtkIdType numberOfPoints, vtkIdType prereservedNumberOfPoints)
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

  INIT_INFO_ARR(Xs                   , "X")
  INIT_INFO_ARR(Ys                   , "Y")
  INIT_INFO_ARR(Zs                   , "Z")
  INIT_INFO_ARR(Azimuths             , "Azimuth")
  INIT_INFO_ARR(VerticalAngles       , "Vertical Angle")
  INIT_INFO_ARR(DistanceTypeStrings  , "Distance Type")
  INIT_INFO_ARR(FiringModeStrings    , "FiringMode")
  INIT_INFO_ARR(StatusStrings        , "Status")
  INIT_INFO_ARR(Intensities          , "Intensity")
  INIT_INFO_ARR(Confidences          , "Confidence")
  INIT_INFO_ARR(Reflectivities       , "Reflectivity")
  INIT_INFO_ARR(ChannelNumbers       , "Logical Channel Number")
  INIT_INFO_ARR(TimeFractionOffsets  , "Time Fraction Offset")
  INIT_INFO_ARR(Powers               , "Power")
  INIT_INFO_ARR(Noises               , "Noise")
  INIT_INFO_ARR(Pseqs                , "Packet Sequence Number")

  return polyData;
}

//------------------------------------------------------------------------------
// TODO: Revisit this if the frequency still needs to be calculated here.
bool VelodyneAdvancedPacketInterpreter::SplitFrame(bool force)
{
  this->FrameSize = std::max(
    this->FrameSize,
    static_cast<decltype(this->FrameSize)>(this->Points->GetNumberOfPoints())
  );
  return this->LidarPacketInterpreter::SplitFrame(force);
}

//------------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::ResetCurrentFrame()
{
  this->CurrentFrame = this->CreateNewEmptyFrame(0);
}

//------------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::PreProcessPacket(unsigned char const * data, unsigned int dataLength, bool & isNewFrame, int & framePositionInPacket)
{
  PacketDataHandle<BYTES_PER_HEADER_WORD> packetDataHandle = PacketDataHandle<BYTES_PER_HEADER_WORD>(data, dataLength, 0);
  Payload<false> payload = Payload<false>(packetDataHandle);
  std::vector<size_t> newFrameBoundaries {0};

  // TODO
  // Review how this is supposed to work. A packet contains a single payload so
  // it's not possible to jump directly to a new frame, and several frames could
  // theoretically be included in a single payload depending on how frames are
  // determined. If the function limits this to one then perhaps we are losing
  // frame data here.
  payload.DetectFrames((* (this->CurrentFrameTracker)), newFrameBoundaries);
  isNewFrame = (newFrameBoundaries.size() > 0);
  framePositionInPacket = 0;
}








//-----------------------------------------------------------------------------
// Code from the legacy packet format interpreter.
//-----------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::InitTrigonometricTables()
{
  if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
  {
    cos_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    sin_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
    {
      double rad = degreesToRadians(i / 100.0);
      cos_lookup_table_[i] = std::cos(rad);
      sin_lookup_table_[i] = std::sin(rad);
    }
  }
}

//-----------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::PrecomputeCorrectionCosSin()
{

  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++)
  {
    HDLLaserCorrection& correction = laser_corrections_[i];
    correction.cosVertCorrection = std::cos(degreesToRadians(correction.verticalCorrection));
    correction.sinVertCorrection = std::sin(degreesToRadians(correction.verticalCorrection));
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

//-----------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::Init()
{
  this->InitTrigonometricTables();
  // this->SensorTransform->Identity()
  this->ResetCurrentFrame();
}

//-----------------------------------------------------------------------------
void VelodyneAdvancedPacketInterpreter::LoadCalibration(const std::string& filename)
{
  boost::property_tree::ptree pt;
  try
  {
    read_xml(filename, pt, boost::property_tree::xml_parser::trim_whitespace);
  }
  catch (boost::exception const&)
  {
    vtkGenericWarningMacro(
      "LoadCalibration: error reading calibration file: " << filename);
    return;
  }
  // Read distLSB if provided
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v, pt.get_child("boost_serialization.DB"))
  {
    if (v.first == "distLSB_")
    { // Stored in cm in xml
      DistanceResolutionM = atof(v.second.data().c_str()) / 100.0;
    }
  }

  int i, j;
  i = 0;
  BOOST_FOREACH (
    boost::property_tree::ptree::value_type& p, pt.get_child("boost_serialization.DB.colors_"))
  {
    if (p.first == "item")
    {
      j = 0;
      BOOST_FOREACH (boost::property_tree::ptree::value_type& v, p.second.get_child("rgb"))
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
    boost::property_tree::ptree::value_type& v, pt.get_child("boost_serialization.DB.enabled_"))
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
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v,
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
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v,
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
    boost::property_tree::ptree::value_type& v, pt.get_child("boost_serialization.DB.points_"))
  {
    if (v.first == "item")
    {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH (boost::property_tree::ptree::value_type& px, points)
      {
        if (px.first == "px")
        {
          boost::property_tree::ptree calibrationData = px.second;
          int index = -1;
          HDLLaserCorrection xmlData;

          BOOST_FOREACH (boost::property_tree::ptree::value_type& item, calibrationData)
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
              xmlData.verticalOffsetCorrection = atof(item.second.data().c_str());
            if (item.first == "horizOffsetCorrection_")
              xmlData.horizontalOffsetCorrection = atof(item.second.data().c_str());
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
              laser_corrections_[index].closeSlope = laser_corrections_[index].focalSlope;
            laser_corrections_[index].minIntensity = minIntensity[index];
            laser_corrections_[index].maxIntensity = maxIntensity[index];
          }
        }
      }
    }
  }

  int idx = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v,
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
  BOOST_FOREACH (boost::property_tree::ptree::value_type& v,
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

//-----------------------------------------------------------------------------
template <typename T>
void VelodyneAdvancedPacketInterpreter::ComputeCorrectedValues(
  T const azimuth,
  FiringReturn<true> const firingReturn,
  size_t const correctionIndex,
  double pos[3]
)
{
  double distance = firingReturn.GetDistance<uint32_t>();
  double cosAzimuth, sinAzimuth;
  HDLLaserCorrection * correction = & (this->laser_corrections_[correctionIndex]);


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
  // Compute the distance in the xy plane (w/o accounting for rotation)
  /**the new term of 'vert_offset * sin_vert_angle'
   * was added to the expression due to the mathemathical
   * model we used.
   */
  double distanceMRaw = distance * this->DistanceResolutionM;
  double distanceM = distanceMRaw + correction->distanceCorrection;
  double xyDistance =
    distanceM * correction->cosVertCorrection - correction->sinVertOffsetCorrection;

  pos[0] = xyDistance * sinAzimuth - correction->horizontalOffsetCorrection * cosAzimuth;
  pos[1] = xyDistance * cosAzimuth + correction->horizontalOffsetCorrection * sinAzimuth;
  pos[2] = distanceM * correction->sinVertCorrection + correction->verticalOffsetCorrection;
}


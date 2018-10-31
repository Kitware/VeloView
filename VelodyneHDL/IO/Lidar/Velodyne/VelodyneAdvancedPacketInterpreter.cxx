#include <type_traits>

#include <boost/preprocessor.hpp>

//------------------------------------------------------------------------------
// Macros.
//------------------------------------------------------------------------------
//! @brief Lengths in the headers are given in units of 32-bit words.
#define BYTES_PER_HEADER_LENGTH_UNIT 4;

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
#define GET_LENGTH(attr) decltype(auto) Get ** attr const { return this->attr * BYTES_PER_HEADER_LENGTH_UNIT; }

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
 *
 * @tparam     TS The source value type.
 * @tparam     TD The destination value type.
 *
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

public:
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
   * @brief Set values from bit sequences that do not align with bytes.
   *
   * This is a wrapper around SetFromBits with automatic deduction of the
   * smallest type required to hold the number of bytes to process.
   *
   * @tparam numberOfBytes   The number of bytes to consume from the input data.
   *                         Values larger than 4 are not supported.
   * @tparam PassthroughArgs All arguments to pass through to SetFromBits.
   *
   * @param[in,out] passthroughArgs All remaining arguments.
   */

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
}

//------------------------------------------------------------------------------
// PayloadHeader
//------------------------------------------------------------------------------
/*!
 * @brief Payload header of the VLP Advanced data packet format.
 *
 * All lengths count 32-bit words, e.g. a Glen value of 4 indicates that the
 * firing group header consists of 4 32-bit words. The raw values are converted
 * to counts with the BYTES_PER_HEADER_LENGTH_UNIT constant.
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


  //! @brief The number of distances in a firing.
  decltype(Dset) DistanceCount;

  //! @brief The number of bytes per distance.
  decltype(Dset) BytesPerDistance;

  //! @brief The number of intensities for a given distance.
  decltype(Iset) IntensityCount;

public:
  //@{
  //! @brief Getters for header values.
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
    auto mask = this->GetDsetMask();
    return (this->IsDsetMask()) ? SET_BITS_IN_BYTE[mask] : mask;

  }

  //! @brief The number of intensities in each firing.
  uint8_t GetIntensityCount const ()
  {
    return SET_BITS_IN_BYTE[this->Iset];
  }


  /*!
   * @brief         Construct a PayloadHeader.
   * @param[in,out] packetData The packet data from which to parse the header.
   */
  PayloadHeader(PacketData & packetData)
  {
    packetData.SetFromBits<1>(
      4, 4, this->Ver,
      0, 4, this->Hlen
    );

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
  }
}


//------------------------------------------------------------------------------
// ExtensionHeader
//------------------------------------------------------------------------------
/*!
 * @brief Extension header of the VLP Advanced data packet format.
 *
 * This should be subclassed to handle different types of extensions as
 * specified by the NXHDR field of the payload header.
 */
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
  //! @brief Getters for header values.
  GET_LENGTH(Hlen)
  GET_RAW(Nxhdr)
  uint8_t const * GetData const { return this->Data; }
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
    packetData.CopyBytes(this->Data, dataLen);
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
   * TOFFS is returned in nanoseconds.
   * HDIR and VDIR are returned as their respective enums.
   * VDFL and AZM are returned in degrees.
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
    if (i < 0 || i >= SET_BITS_IN_BYTE[this->MyFiring.MyFiringGroup.MyPayload.GetHeader().GetIset()])
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
    auto icount        = this->MyFiring.MyFiringGroup.MyPayload.GetHeader().GetIntensityCount();
    auto bytesPerValue = this->MyFiring.MyFiringGroup.MyPayload.GetHeader().GetBytesPerDistance();
    packetData.CopyBytes(this->Data, (icount * bytesPerValue));
  }
};

//------------------------------------------------------------------------------
// Firing
//------------------------------------------------------------------------------
//! @brief Contains a firing header and a variable number of firing returns.
class Firing
{
private:
  //! @brief Firing header.
  FiringHeader Header;

  //! @brief Returns in this firing.
  std::vector<FiringReturn> Returns;

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Header);
  GET_CONST_REF(Returns);
  //@}

  //! @brief Reference to this firing's group.
  FiringGroup const & MyFiringGroup;

  Firing(FiringGroup const & firingGroup, PacketData & packetData)
    : MyFiringGroup = firingGroup
  {
    this->Header = FiringHeader(packetData);

    auto dcount = this->MyFiringGroup.MyPayload.GetHeader().GetDistanceCount();
    this->returns.reserve(dcount);
    for (decltype(dcount) j = 0; j < dcount; ++j)
    {
      FiringReturn firingReturn = FiringReturn((* this), packetData);
      this->returns.push_back(firingReturn);
    }
  }
}

//------------------------------------------------------------------------------
// FiringGroup
//------------------------------------------------------------------------------
class FiringGroup
{
private:
  //! @brief Firing group header.
  FiringGroupHeader Header;

  //! @brief Firings.
  std::vector<Firing> Firings {0};

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Header);
  GET_CONST_REF(Firings);
  //@}

  //! @brief Reference to the payload containing this firing group.
  Payload const & MyPayload;

  /*!
   * @param[in]     payload   The payload that contains this firing group.
   * @param[in]     data       Pointer to the packet bytes.
   * @param[in]     dataLength The byte length of the packet data.
   * @param[in,out] i          The offset to the data.
   */
  FiringGroup(Payload const & payload, PacketData & packetData)
    : MyPayload = payload
  {
    // Group headers are padded to 32-bit boundaries. Use GLEN to advance the
    // index correctly.
    auto glen = this->MyPayload->GetGlen();
    size_t index = packetData.Index;
    this->Header = FiringGroupHeader(packetData);
    packetData.Index = index + glen;

    auto fcnt = this->header.GetFcnt();
    this->Firings.reserve(fcnt);

    for (decltype(fcnt) j = 0; j < fcnt; ++j)
    {
      Firing firing = Firing((* this), packetData);
      this->Firings.push_back(firing);
    }
  }
}

//------------------------------------------------------------------------------
// Payload
//------------------------------------------------------------------------------
class Payload
{
private:
  //! @brief Payload header.
  PayloadHeader Header;

  //! @brief Variable number of extension headers.
  std::vector<ExtensionHeader> ExtensionHeaders {0};

  //! @brief Variable number of firing groups.
  std::vector<FiringGroup> FiringGroups {0};

public:
  //@{
  //! @brief Getters.
  GET_CONST_REF(Header);
  GET_CONST_REF(ExtensionHeaders);
  GET_CONST_REF(FiringGroups);
  //@}

  Payload(PacketData & packetData)
  {
    this->header = PayloadHeader(packetData);

    // Check for extension headers.
    auto nxhdr = this->header.GetNxhdr();
    while (nxhdr != 0)
    {
      // The extension header automatically adjusts its length to end on a
      // 32-bit boundary so padding need not be handled here.
      ExtensionHeader extensionHeader = ExtensionHeader(packetData);
      this->ExtensionHeaders.push_back(extensionHeader);
      nxhdr = extensionHeader.GetNxhdr();
    }

    // The rest of the data should be filled with firing groups.
    while (i < dataLength)
    {
      FiringGroup firingGroup = FiringGroup(this, packetData);
      this->FiringGroups.push_back(firingGroup);
    }
  }
}




//------------------------------------------------------------------------------
// ProcessData
//------------------------------------------------------------------------------
void ProcessPacket(unsigned char const * data, unsigned int dataLength, int startPosition)
{
  size_t offset = 0;
  // Given that startPosition is a signed int, just add it to the data pointer
  // and be done with it.
  data += startPosition;

  // The packet classes throw length errors if the packet does not contain the
  // expected length.
  try
  {
    PayloadHeader payloadHeader = PayloadHeader(data, dataLength, offset);
  }
  catch (const std::length_error & e)
  {
  }
}


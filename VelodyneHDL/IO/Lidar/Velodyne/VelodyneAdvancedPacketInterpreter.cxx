#include <type_traits>

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
//------------------------------------------------------------------------------
//! @brief Model identification code.
enum ModelIdentificationCode {
  MIC_RESERVED = 0,
  MIC_VLP16    = 1,
  MIC_VLP16_HD = 2,
  MIC_VLP32A   = 3,
  MIC_VLP32B   = 4
};

//------------------------------------------------------------------------------
//! @brief Horizontal direction of Lidar.
enum HorizontalDirection {
  HD_CLOCKWISE         = 0,
  HD_COUNTER_CLOCKWISE = 1
};

//------------------------------------------------------------------------------
//! @brief Vertical direction of Lidar.
enum VerticalDirection {
  VD_UP   = 0,
  VD_DOWN = 1
}

//------------------------------------------------------------------------------
//! @brief Firing mode.
enum FiringMode {
  FM_PASSIVE = 0,
  FM_NORMAL  = 1,
  FM_RESERVED = 2, // 2-14 are reserved
  FM_INDEX = 15
}

//------------------------------------------------------------------------------
//! @brief Channel status flags.
enum ChannelStatus
{
  CS_OBSTRUCTION_DETECTED = 0,
  CS_FAULT_DETECTED       = 1,
  CS_POWER_NOT_PERFECT    = 2,
  CS_RESERVED             = 3
}

//------------------------------------------------------------------------------
//! @brief Mask format to specify number and type of values in intensity set.
enum IntensityType
{
  IM_REFLECTIVITY = 0,
  IM_INTENSITY    = 1 << 0,
  IM_CONFIDENCE   = 1 << 1,
  IM_RESERVED     = 1 << 2 // bits 3-15 are reserved
}

//------------------------------------------------------------------------------
//! @brief Mask format to specify values in returned distance set.
enum DistanceType
{
  DM_FIRST            = 0,
  DM_STRONGEST        = 1 << 0,
  DM_SECOND_STRONGEST = 1 << 1,
  DM_LAST             = 1 << 2,
  DM_RESERVED         = 1 << 3 // bits 4-5 are reserved
}

//------------------------------------------------------------------------------
// Macros.
//------------------------------------------------------------------------------
//! @brief Lengths in the headers are given in units of 32-bit words.
#define BYTES_PER_HEADER_LENGTH_UNIT 4;

//! @brief Simple getter for uninterpretted values.
#define GET_RAW(attr) decltype(auto) Get ## attr const { return this->attr; }

//! @brief Get a header length in bytes.
#define GET_LENGTH(attr) decltype(auto) Get ** attr const { return this->attr * BYTES_PER_HEADER_LENGTH_UNIT; }

#define GET_HEADER decltype(auto) const & GetHeader const { return this->Header; }

//------------------------------------------------------------------------------
// Convenience functions.
//------------------------------------------------------------------------------
/*!
 * @brief Set a value from a byte sequence.
 * @todo Add endianness detection and optimize accordingly.
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
 * @brief Set a target value from a range of bits in another value.
 *
 * @tparam TS The source value type.
 * @tparam TD The destination value type.
 *
 * @param[in]  source The source value from which to set the destination value.
 * @param[in]  offset The offset of the least significant bit.
 * @param[in]  number The number of bits to set.
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
 * @brief 
 * Variadic overload to handle variable number of destination values (repeated
 * triples of offset, number and destination).
 *
 * @todo Check if this has any noticeable performance penalty.
 */
template <typename TS, typename TD, typename... RemainingArgs>
inline
void setFromBits(TS & source, uint8_t offset, uint8_t number, TD & destination, RemainingArgs... remainingArgs)
{
  setFromBits(source, offset, number, destination);
  setFromBits(source, remainingArgs);
}

//------------------------------------------------------------------------------
//! @brief Convencience struct for processing packet data.
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
  //! @brief Get the number of bytes remaining from the current index.
  size_t GetRemainingLength()
  {
    return (this->Length > this-Index) ? (this->Length - this->Index) : 0;
  }

  //! @brief Set a 1-byte value.
  template <typename T>
  void SetFromByte(T & value)
  {
    value = this->Data[(this->Index)++];
  }

  //! @brief Set a multi-byte value (wraps setFromBytes).
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
   * @tparam numberOfBytes The number of bytes to consume from the input data.
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
   * @brief Copy a sequence of bytes.
   * @param[out] data  The destination to which to copy the data.
   * @param[in] length The number of bytes to copy.
   */
  void CopyBytes(uint8_t * & data, size_t length)
  {
    data = new uint8_t[length];
    std::memcpy(data, this->Data[this->Index], length);
    this->Index += length;
  }
}

//------------------------------------------------------------------------------
// PayloadHeader
//------------------------------------------------------------------------------
/*!
 * @brief Payload header of the VLP Advanced data packet format.
  Elle en aura besoin pour développer son algo de SLAM*
 * All lengths count 32-bit words, e.g. a Glen value of 4 indicates that the
 * firing group header consists of 4 32-bit words.
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

  // TODO
  // Dset, Iset and Tref all require interpretation.

  //! @brief Getters for header values.

  GET_RAW(Ver)
  GET_LENGTH(Hlen)
  GET_RAW(Nxhdr)
  GET_LENGTH(Glen)
  GET_LENGTH(Flen)
  
  ModelIdentificationCode GetMic const ()
  {
    switch (this->Mic)
    {
      case 1:
        return MIC_VLP16;
      case 2:
        return MIC_VLP16_HD;
      case 3:
        return MIC_VLP32A;
      case 4:
        return MIC_VLP32B;
      // case 0:
      default:
        return MIC_RESERVED;
    }
  }

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
   * @brief Construct a PayloadHeader.
   * @param[in] data The packet data.
   * @param[in,out] i
   *   The offset to the start of the header. The offset will be advanced as the
   *   data is consumed.
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
    * extension-specific and determined by the NXHDR value of the payload
    * header.
    */
  uint8_t * Data;

public:
  //@{
  //! @brief Getters for header values.
  GET_LENGTH(Hlen)
  GET_RAW(Nxhdr)
  uint8_t const * GetData const { return this->Data; }
  //@}

  /*!
   * @brief Construct a ExtensionHeader.
   * @param[in] data The packet data.
   * @param[in,out] i
   *   The offset to the start of the header. The offset will be advanced as the
   *   data is consumed.
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

  ~ExtensionHeader()
  {
    delete[] this->Data;
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
  //! @brief Getters for header values.
  uint8_t GetFcnt const { return this->Fcnt + 1; }
  uint8_t GetFspn const { return this->Fspn + 1; }
  GET_RAW(Fdly)
  HorizontalDirection GetHdir const { return this->Hdir; }
  VerticalDirection   GetVdir const { return this->Vdir; }
  double GetVdfl const { return this->Vdfl * 0.01; }
  double GetAzm const { return this->Azm * 0.01; }
  //! @brief Get the TOFFS in nanoseconds.
  uint32_t GetToffs const {
    return static_cast<uint32_t>(this->Toffs) * 64;
  }
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
  //! @brief Getters for header values.
  GET_RAW(Lcn)

  FiringMode GetFm const ()
  {
    switch (this->Fm)
    {
      case 0:
        return FM_PASSIVE;
      case 1:
        return FM_NORMAL;
      case 15:
        return FM_INDEX;
      default:
        return FM_RESERVED;
    }
  }
  GET_RAW(Pwr)
  GET_RAW(Nf)

  ChannelStatus GetStat const ()
  {
    switch (this->Stat)
    {
      case 0:
        return CS_OBSTRUCTION_DETECTED;
      case 1:
        return CS_FAULT_DETECTED;
      case 2:
        return CS_POWER_NOT_PERFECT;
      default:
        return CS_RESERVED;
    }
  }

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
  decltype(PacketData::Data) Data;

public:
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
    if (! (this->MyFiring->MyFiringGroup->MyPayload->GetIset() & type))
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

  ~FiringReturn()
  {
    delete[] this->Data;
  }
};

//------------------------------------------------------------------------------
// Firing
//------------------------------------------------------------------------------
class Firing
{
private:
  //! @brief Firing header.
  FiringHeader Header;

  //! @brief Returns in this firing.
  std::vector<FiringReturn> returns;

public:
  GET_HEADER

  //! @brief Reference to this firing's group.
  FiringGroup const & MyFiringGroup;

  Firing(FiringGroup const & firingGroup, PacketData & packetData)
    : MyFiringGroup = firingGroup
  {
    this->Header = FiringHeader(packetData);

    auto dcount = this->MyFiringGroup.MyPayload.GetHeader()->GetDistanceCount();
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
  GET_HEADER

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
  std::vector<ExtensionHeader> extensionHeaders {0};

  //! @brief Variable number of firing groups.
  std::vector<FiringGroup> firingGroups {0};

public:
  GET_HEADER

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
      this->extensionHeaders.push_back(extensionHeader);
      nxhdr = extensionHeader.GetNxhdr();
    }

    // The rest of the data should be filled with firing groups.
    while (i < dataLength)
    {
      FiringGroup firingGroup = FiringGroup(this, packetData);
      this->firingGroups.push_back(firingGroup);
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


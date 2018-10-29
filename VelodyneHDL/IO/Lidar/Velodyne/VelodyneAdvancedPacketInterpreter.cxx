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
enum isetMask
{
  IM_REFLECTIVITY = 0,
  IM_INTENSITY    = 1 << 0,
  IM_CONFIDENCE   = 1 << 1,
  IM_RESERVED     = 1 << 2 // bits 3-15 are reserved
}

//------------------------------------------------------------------------------
//! @brief Mask format to specify values in returned distance set.
enum dsetMask
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
#define GET_RAW(attr) decltype(auto) get ## attr const { return this->attr; }

//! @brief Get a header length in bytes.
#define GET_LENGTH(attr) decltype(auto) get ** attr const { return this->attr * BYTES_PER_HEADER_LENGTH_UNIT; }

//------------------------------------------------------------------------------
// Convenience functions.
//------------------------------------------------------------------------------
/*!
 * @brief Set a value from a byte sequence.
 * @todo Add endianness detection and optimize accordingly.
 * @param[in]     bytes         The array of input bytes.
 * @param[in,out] i             The index of the first byte to read.
 * @param[out]    value         The output value.
 * @param[in]     numberOfBytes Number of bytes to use for the value.
 */
template <typename S, typename T>
inline
void set_from_bytes(uint8_t const * bytes, S & i, T & value, size_t numberOfBytes = sizeof(T))
{
  S stop = i + numberOfBytes;
  value = bytes[i++];
  while (i < stop)
  {
    value = bytes[i++] + (value * (1 << sizeof(uint8_t)));
  }
}

//------------------------------------------------------------------------------
/*!
 * @brief Set a target value from a range of bits in another value.
 *
 * @tparam T    The value type.
 *
 * @param[in]     value  The value from which to set the targets.
 * @param[in]     lsb    The least significant bit of the range for the first target.
 * @param[in]     msb    The most significant bit of the range for the first target.
 * @param[out]    target The target to set from the range [lsb-msb] of the value.
 */
template <typename T>
inline
void set_from_bits(T & value, uint8_t offset, uint8_t number, T & target)
{
  T const mask = (1 << number) - 1;
  target = (value >> offset) & mask;
  set_bits(value, args);
}

//------------------------------------------------------------------------------
// PayloadHeader
//------------------------------------------------------------------------------
/*!
 * @brief Payload header of the VLP Advanced data packet format.
 *
 * All lengths count 32-bit words, e.g. a Glen value of 4 indicates that the
 * firing group header consists of 4 32-bit words.
 */
class PayloadHeader
{
private:
  //! @brief Protocol version.
  uint8_t Ver;

  //! @brief Header length (min: 6)
  uint8_t Hlen;

  //! @brief Next header type.
  uint8_t Nxhdr;

  //! @brief Firing group header length.
  uint8_t Glen;

  //! @brief Firing header length.
  uint8_t Flen;

  //! @brief Model Identification Code
  ModelIdentificationCode Mic;

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
  //! @brief Getters for header values.

  GET_RAW(Ver);
  GET_LENGTH(Hlen);
  GET_RAW(Nxhdr);
  GET_LENGTH(Glen);
  GET_LENGTH(Flen);
  GET_RAW(Mic);
  GET_RAW(Tstat);

  // TODO
  // Dset, Iset and Tref all require interpretation.

  GET_RAW(Pset);

  //@}

  /*!
   * @brief Construct a PayloadHeader.
   * @param[in] data The packet data.
   * @param[in,out] i
   *   The offset to the start of the header. The offset will be advanced as the
   *   data is consumed.
   */
  PayloadHeader(uint8_t const * const data, size_t length, size_t & i)
  {
    size_t remainingLength = length - i;

    // Note the post-increment operators here.
    uint8_t verHlen = data[i++];
    set_from_bits(verHlen, 4, 4, this->Ver);
    set_from_bits(verHlen, 0, 4, this->Hlen);

    if (this->getHlen()> remainingLength)
    {
      raise std::length_error("data does not contain enough bytes for header");
    }

    this->Nxhdr = data[i++];

    uint8_t glenFlen = data[i++];
    set_from_bits(glenFlen, 4, 4, this->Glen);
    set_from_bits(glenFlen, 0, 4, this->Flen);

    this->Mic = static_cast<decltype(this->Mic)>(data[i++]);
    this->Tstat = data[i++];
    this->Dset = data[i++];
    this->Iset = data[i++];

    // set_from_bytes increments i by the size of the third argument.
    set_from_bytes(data, i, this->Tref);
    set_from_bytes(data, i, this->Pseq);
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
  uint8_t * data;

public:
  //@{
  //! @brief Getters for header values.
  GET_LENGTH(Hlen);
  GET_RAW(Nxhdr);
  uint8_t const * const getData const { return this->Data; }
  //@}

  /*!
   * @brief Construct a ExtensionHeader.
   * @param[in] data The packet data.
   * @param[in,out] i
   *   The offset to the start of the header. The offset will be advanced as the
   *   data is consumed.
   */
  ExtensionHeader(uint8_t const * data, size_t length, size_t & i)
  {
    size_t remainingLength = length - i;
    this->Hlen = data[i++];

    auto hlen = this->getHlen();

    if (hlen > remainingLength)
    {
      raise std::length_error("data does not contain enough bytes for header");
    }

    this->Nxhdr = data[i++];

    auto dataLen = hlen - (sizeof(Hlen) + sizeof(Nxhdr));
    // The length is in units of 32-bit/4-byte words.
    this->data = new uint8_t[dataLen];
    std::memcpy(this->data, data[i], dataLen);
    i += dataLen;
  }

  ~ExtensionHeader()
  {
    delete[] this->data;
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
  uint8_t getFcnt const { return this->Fcnt + 1; }
  uint8_t getFspn const { return this->Fspn + 1; }
  GET_RAW(Fdly);
  HorizontalDirection getHdir const { return this->Hdir; }
  VerticalDirection   getVdir const { return this->Vdir; }
  double getVdfl const { return this->Vdfl * 0.01; }
  double getAzm const { return this->Azm * 0.01; }
  //@}

  /*!
   * @brief Construct a FiringGroupHeader.
   * @param[in] data The packet data.
   * @param[in,out] i
   *   The offset to the start of the header. The offset will be advanced as the
   *   data is consumed.
   */
  FiringGroupHeader(uint8_t const * data, size_t & i)
  {
    // set_from_bytes increments i by the size of the third argument.
    set_from_bytes(data, i, this->Toffs);

    uint8_t fcntFspn = data[i++];
    set_from_bits(fcntFspn, 3, 5, this->Fcnt);
    set_from_bits(fcntFspn, 0, 3, this->Fspn);

    this->Fdly = data[i++];

    uint16_t hdirVdirVdfl;
    set_from_bytes(data, i, hdirVdirVdfl);
    set_from_bits(hdirVdirVdfl, 15,  1, this->Hdir);
    set_from_bits(hdirVdirVdfl, 14,  1, this->Vdir);
    set_from_bits(hdirVdirVdfl,  0, 14, this->Vdfl);

    set_from_bytes(data, i, this->Azm;
  }

  //! @brief Get the TOFFS in nanoseconds.
  uint32_t getToffs const {
    return static_cast<uint32_t>(this->Toffs) * 64;
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
  GET_RAW(Lcn);

  FiringMode getFm const ()
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
  GET_RAW(Pwr);
  GET_RAW(Nf);

  ChannelStatus getStat const ()
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
  FiringHeader(uint8_t const * data, size_t & i)
  {
    this->Lcn = data[i++];

    // set_from_bytes increments i by the size of the third argument.
    uint8_t fmPwr;
    set_from_bytes(data, i, fmPwr);
    set_from_bits(fmPwr, 4, 4, this->Fm);
    set_from_bits(fmPwr, 0, 4, this->Pwr);

    this->Nf = data[i++];
    this->Stat = data[i++];
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
  //! @brief The firing to which this return belongs.
  Firing * firing;

  //! @brief The firing distance.
  uint32_t Distance;

  //! @brief Intensities
  uint32_t * Intensities;

public:
  FiringReturn(Firing * firing_, uint8_t const * const data, size_t & i)
    : firing = firing_
  {
    auto icount = this->firing->firingGroup->payload->getIcount();
    auto bytesPerDistance = this->firing->firingGroup->payload->getBytesPerDistance();

    set_from_bytes(data, i, this->Distance, bytesPerValue);
    this->Intensities = new T[icount];
    for (uint8_t i = 0; i < icount; ++i)
    {
      set_from_bytes(data, i, this->Intensities[i], encodingSizeInBytes);
    }
  }

  ~FiringReturn()
  {
    delete[] this->Intensities;
  }
};

//------------------------------------------------------------------------------
// Firing
//------------------------------------------------------------------------------
class Firing
{
private:
  //! @brief The firing group to which this firing belongs.
  FiringGroup * firingGroup

  //! @brief Firing header.
  FiringHeader header;

  //! @brief Returns in this firing.
  std::vector<FiringReturn> returns;

public:
  Firing(
    FiringGroup * firingGroup_,
    uint8_t const * const data, size_t dataLength, size_t & i
  )
    : firingGroup = firingGroup_
  {
    auto dcount = this->firingGroup->payload->getDcount();
    this->returns.reserve(dcount);
    for (decltype(dcount) j = 0; j < dcount; ++j)
    {
      FiringReturn firingReturn = FiringReturn(this, data, dataLength, i);
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
  //! @brief Pointer to parent payload.
  Payload * payload;

  //! @brief Firing group header.
  FiringGroupHeader header;

  //! @brief Firings.
  std::vector<Firing> firings {0};

public:
  /*!
   * @param[in]     payload_   The payload that contains this firing group.
   * @param[in]     data       Pointer to the packet bytes.
   * @param[in]     dataLength The byte length of the packet data.
   * @param[in,out] i          The offset to the data.
   */
  FiringGroup(
    Payload const * payload_,
    uint8_t const * const data, size_t dataLength, size_t & i
  )
    : payload = payload_;
  {
    // Group headers are padded to 32-bit boundaries. Use GLEN to advance the
    // index correctly.
    auto glen = payload->getGlen();
    size_t tmp_i = i;
    this->header = FiringGroupHeader(data, dataLength, tmp_i);
    i += glen;

    auto fcnt = this->header.getFcnt();
    this->firings.reserve(fcnt);

    for (decltype(fcnt) j = 0; j < fcnt; ++j)
    {
      Firing firing = Firing(this, data, dataLength, i);
      this-firing.push_back(firing);
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
  PayloadHeader header;

  //! @brief Variable number of extension headers.
  std::vector<ExtensionHeader> extensionHeaders {0};

  //! @brief Variable number of firing groups.
  std::vector<FiringGroup> firingGroups {0};

  //! @brief The number of distances in a firing.
  decltype(PayloadHeader::Dset) Dcount;

  //! @brief The number of bytes per distance.
  decltype(PayloadHeader::Dset) BytesPerDistance;

  //! @brief The number of intensities for a given distance.
  decltype(PayloadHeader::Iset) Icount;

public:
  //@{
  //! @brief Getters.
  GET_RAW(Dcount);
  GET_RAW(BytesPerDistance);
  GET_RAW(Icount);
  //@}

  Payload(uint8_t const * const data, size_t dataLength, size_t & i)
  {
    this->header = PayloadHeader(data, dataLength, i);

    // Calculate dset- and iset-derived values once here to avoid redundant
    // calculations later.
    auto dset = this->header.getDset();
    decltype(dset) dcount = ((1 << 6) - 1) & dset;
    bool is_count = dset & (1 << 6);
    // If not a count, then each set bit indicates the presence of a value.
    if (! is_count)
    {
      dcount = SET_BITS_IN_BYTE[dcount];
    }
    this->Dcount = dcount;
    this->BytesPerDistance = (dset & (1 << 7)) ? 3 : 2;

    auto iset = this->header.getIset();
    this->Icount = SET_BITS_IN_BYTE[iset];


    // Check for extension headers.
    auto nxhdr = this->header.getNxhdr();
    while (nxhdr != 0)
    {
      // The extension header automatically adjusts its length to end on a
      // 32-bit boundary so padding need not be handled here.
      ExtensionHeader extensionHeader = ExtensionHeader(data, dataLength, i);

      this->extensionHeaders.push_back(extensionHeader);
      nxhdr = extensionHeader.getNxhdr();
    }


    while (i < dataLength)
    {
      FiringGroup firingGroup = FiringGroup(this, data, dataLength, i);
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


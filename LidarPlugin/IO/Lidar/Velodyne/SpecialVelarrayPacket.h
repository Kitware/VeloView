#ifndef SPECIALVELARRAYPACKET_H
#define SPECIALVELARRAYPACKET_H

#include <boost/predef/detail/endian_compat.h>
#include <boost/endian/arithmetic.hpp>
#include <memory>

//----------------------------------------------------------------------------
//! @brief Simple getter that handles conversion to native unsigned integer types.
#define GET_NATIVE_UINT(n, attr) uint ## n ##_t Get ## attr() const { return this->attr; }
#define SET_NATIVE_UINT(n, attr) void Set ## attr(uint ## n ##_t x) { this->attr = x; }

#define BIT(n)                  ( 1<<(n) )
//! Create a bitmask of length \a len.
#define BIT_MASK(len)           ( BIT(len)-1 )

//! Create a bitfield mask of length \a starting at bit \a start.
#define BF_MASK(start, len)     ( BIT_MASK(len)<<(start) )

//! Extract a bitfield of length \a len starting at bit \a start from \a y.
#define BF_GET(y, start, len)   ( ((y)>>(static_cast<decltype(y)>(start))) & BIT_MASK(len) )

//! Prepare a bitmask for insertion or combining.
#define BF_PREP(x, start, len)  ( ((x)&BIT_MASK(len)) << (start) )

//! Insert a new bitfield value x into y.
#define BF_SET(y, x, start, len)    ( y= ((y) &~ BF_MASK(start, len)) | BF_PREP(x, start, len) )


namespace SpecialVelarrayPacket {
#pragma pack(push, 1)
//! @brief class representing the Special Velarray packet header
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |   VER  | HLEN  |    NXHDR      | PTYPE | TLEN  |      MIC     |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                               PSEQ                            |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |                               TREF                            |
  |                                                               |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |  GLEN  | FLEN  |     DSET      |             ISET             |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
class PayloadHeader
{
private:
  // 4 bits VER : Reserved, 0x1
  // 4 bits HLEN : Reserved, 0x5
  boost::endian::big_uint8_t VER_HLEN;

  // NXHDR : Reserved, 0x00
  boost::endian::big_uint8_t NXHDR;

  // 4 bits PTYPE : Reserved, 0x1
  // 4 bits TLEN : Reserved, 0x1
  boost::endian::big_uint8_t PTYPE_TLEN;

  // MIC : Model Identification Code (identify the sensor model). MIC for Velarray is 0x40.
  boost::endian::big_uint8_t MIC;

  // PSEQ : Payload Sequence Number. Monotonically increasing sequence starting at zero when the sensor is started / reset and incrementing with each payload sent
  boost::endian::big_uint32_t PSEQ;

  // TREF : Time Reference. Reference timestamp in PTP truncated (64-bit) format.
  boost::endian::big_uint64_t TREF;

  // 4 bits GLEN : Reserved, 0x1
  // 4 bits FLEN : Reserved, 0x0
  boost::endian::big_uint8_t GLEN_FLEN;

  // DSET : Reserved, 0x02
  boost::endian::big_uint8_t DSET;

  //ISET : Reserved, 0x0001
  boost::endian::big_uint16_t ISET;

public:
  GET_NATIVE_UINT(8, NXHDR)
  SET_NATIVE_UINT(8, NXHDR)
  GET_NATIVE_UINT(8, MIC)
  SET_NATIVE_UINT(8, MIC)
  GET_NATIVE_UINT(32, PSEQ)
  SET_NATIVE_UINT(32, PSEQ)
  GET_NATIVE_UINT(64, TREF)
  SET_NATIVE_UINT(64, TREF)
  GET_NATIVE_UINT(8, DSET)
  SET_NATIVE_UINT(8, DSET)

  uint8_t GetVER() const
  {
    return BF_GET(VER_HLEN, 4, 4);
  }

  uint8_t GetHLEN() const
  {
    return BF_GET(VER_HLEN, 0, 4);
  }

  uint8_t GetPTYPE() const
  {
    return BF_GET(PTYPE_TLEN, 4, 4);
  }

  uint8_t GetTLEN() const
  {
    return BF_GET(PTYPE_TLEN, 0, 4);
  }

  uint8_t GetGLEN() const
  {
    return BF_GET(GLEN_FLEN, 4, 4);
  }

  uint8_t GetFLEN() const
  {
    return BF_GET(GLEN_FLEN, 0, 4);
  }

};
#pragma pack(pop)


#pragma pack(push, 1)
//! @brief class representing the Special Velarray firing return
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |HDIR|VDIR|         VDFL         |              AZM             |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |               DIST             |      RFT      |      LCN     |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
class FiringReturn
{
private:
  // 1 bit HDIR : Horizontal Direction (0:Clockwise / 1:Counter-clockwise)
  // 1 bit VDIR : Vertical direction (0:Upward / 1:Downward)
  // 14 bits VDFL : Vertical deflection angle (0.01 degree increments) [0..16383]
  boost::endian::big_uint16_t HDIR_VDIR_VDFL;

  // AZM : Azimuth (0.01 degree increments) [0..35999]
  boost::endian::big_uint16_t AZM;

  // DIST : Distance Return
  boost::endian::big_uint16_t DIST;

  // RFT : Reflectivity
  boost::endian::big_uint8_t RFT;

  // LCN : Logical Channel Number
  boost::endian::big_uint8_t LCN;

public:
  GET_NATIVE_UINT(16, AZM)
  SET_NATIVE_UINT(16, AZM)
  GET_NATIVE_UINT(16, DIST)
  SET_NATIVE_UINT(16, DIST)
  GET_NATIVE_UINT(8, RFT)
  SET_NATIVE_UINT(8, RFT)
  GET_NATIVE_UINT(8, LCN)
  SET_NATIVE_UINT(8, LCN)

  uint8_t GetHDIR() const
  {
    return BF_GET(HDIR_VDIR_VDFL, 15, 1);
  }
  void SetHDIR(uint8_t hdir)
  {
    BF_SET(HDIR_VDIR_VDFL, hdir, 15, 1);
  }
  uint8_t GetVDIR() const
  {
    return BF_GET(HDIR_VDIR_VDFL, 14, 1);
  }
  void SetVDIR(uint8_t vdir)
  {
    BF_SET(HDIR_VDIR_VDFL, vdir, 14, 1);
  }
  uint16_t GetVDFL() const
  {
    return BF_GET(HDIR_VDIR_VDFL, 0, 14);
  }
  void SetVDFL(uint16_t vdfl)
  {
    BF_SET(HDIR_VDIR_VDFL, vdfl, 0, 14);
  }
};
#pragma pack(pop)


#pragma pack(push, 1)
//! @brief class representing the Special Velarray packet footer
/*
   0               1               2               3
   0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7 0 1 2 3 4 5 6 7
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  |               CRC              |      AC       |     PSEQF    |
  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
class PayloadFooter
{
private :
  // CRC : Cyclic Redundancy Check
  boost::endian::big_uint16_t CRC;

  // AC : Alive Counter. Monotonically increasing sequence starting at zero when the sensor is started / reset and incrementing with each payload sent
  boost::endian::big_uint8_t AC;

  // PSEQF : Payload Sequence Number within a Frame. When a new frame starts, this value will reset to zero.
  boost::endian::big_uint8_t PSEQF;

public :
  GET_NATIVE_UINT(16, CRC)
  SET_NATIVE_UINT(16, CRC)
  GET_NATIVE_UINT(8, AC)
  SET_NATIVE_UINT(8, AC)
  GET_NATIVE_UINT(8, PSEQF)
  SET_NATIVE_UINT(8, PSEQF)
};
#pragma pack(pop)

#pragma pack(push, 1)
//! @brief class representing the Special Velarray Packet
struct Packet{
  PayloadHeader header;
  FiringReturn firings[];

  const PayloadFooter* GetFooter(size_t packet_size) const
  {
    long last_firing_id = FiringCount(packet_size) - 1;
    return reinterpret_cast<const PayloadFooter*>(&(this->firings[last_firing_id + 1]));
  }
  static long FiringCount(long bytes)
  {
    return (bytes - sizeof(PayloadHeader) - sizeof(PayloadFooter)) / sizeof(FiringReturn);
  }
};
#pragma pack(pop)

}


#endif // SPECIALVELARRAYPACKET_H

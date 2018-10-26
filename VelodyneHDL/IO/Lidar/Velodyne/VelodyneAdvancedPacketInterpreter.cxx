//! @brief The type of a byte.
typedef uint8_t byte_t;


/*!
 * @brief Set a value for each half of a type.
 * @param[in]  input The input value.
 * @param[out] msb   The value to set from the most significant bits.
 * @param[out] lsb   The value to set from the least significant bits.
 */
template <typename T>
inline
void set_halves(T const input, T & msb, T & lsb)
{
  byte_t const halfSize = sizeof(T) / 2;
  T const mask = (1 << halfSize) - 1;
  msb = (input >> halfSize) & mask;
  lsb = input & mask;
}

/*!
 * @brief Set a value from a byte sequence.
 * @todo Add endianness detection and optimize accordingly.
 * @param[in] bytes The array of input bytes.
 * @param[in,out] i The index of the first byte to read.
 * @param[out] value The output value.
 */
template <typename S, typename T>
inline
void set_from_bytes(byte_t const * bytes, S & i, T & value)
{
  S stop = i + sizeof(T);
  value = bytes[i++];
  while (i < stop)
  {
    value = bytes[i++] + (value * (1 << sizeof(byte_t)));
  }
}

enum ModelIdentificationCode {
  Reserved = 0,
  VLP16    = 1,
  VLP16_HD = 2,
  VLP32A   = 3,
  VLP32B   = 4
};

/*!
 * @brief
 * A class to parse all information from the VLP Advanced data packet header.
 *
 * All lengths count 32-bit words, e.g. a Glen value of 4 indicates that the
 * firing group header consists of 4 32-bit words.
 */
class PayloadHeader
{
private:
  //! @brief Protocol version.
  byte_t Ver;

  //! @brief Header length (min: 6)
  byte_t Hlen;

  //! @brief Next header type.
  byte_t Nxhdr;

  //! @brief Firing group header length.
  byte_t Glen;

  //! @brief Firing header length.
  byte_t Flen;

  //! @brief Model Identification Code
  ModelIdentificationCode Mic;

  //! @brief Time status (TBD).
  byte_t Tstat;

  //! @brief Distance set.
  byte_t Dset;

  //! @brief Intensity set.
  byte_t Iset;

  //! @brief Time reference.
  uint64_t Tref;

  //! @brief Payload sequence number.
  uint32_t Pset;


public:
  PayloadHeader(byte_t const * data)
  {
    unsigned int i = 0;

    set_halves(data[i++], this->Ver, this->Hlen);
    this->Nxhdr = data[i++];
    set_halves(data[i++], this->Glen, this->Flen);
    this->Mic = static_cast<decltype(this->Mic)>(data[i++]);
    this->Tstat = data[i++];
    this->Dset = data[i++];
    this->Iset = data[i++];
    set_from_bytes(data, i, this->Tref);
    set_from_bytes(data, i, this->Pseq);
  }
}

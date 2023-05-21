
// # The crystal oscillator frequency and frequency synthesizer step size.
// # See the datasheet for details of this calculation.
const float _FXOSC = 32000000.0;
const float _FSTEP = _FXOSC / 524288;


class _RegisterBits{
    public:
    uint8_t address;
    uint8_t mask;
    uint8_t offset;
    _RegisterBits(uint8_t _address, uint8_t _offset,uint8_t bits = 1){
        uint8_t i=0;
        mask=0;
        //TODO: check offset to be [0,7] and bits [1,8]
        address = _address;
        for (i=0;i<bits;i++){
            mask<<=1;
            mask|=1;
        }
        mask <<= _offset;
        offset = _offset;
        }
    void set(uint8_t val){
        uint8_t regVal = spi_read_u8(address);
        regVal &= ~mask;
        regVal |= (val & 0xFF) << offset;
        spi_write_u8(address,regVal);
    }
    uint8_t get(){
        uint8_t regVal = spi_read_u8(address);
        return ((regVal & mask) >> offset);
    }
    uint8_t debug(){
      Serial.print ("Mask: ");
      Serial.println(mask,BIN);
      return mask;     
  
    }
};


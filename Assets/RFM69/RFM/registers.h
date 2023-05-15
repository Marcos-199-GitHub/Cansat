
//Internal constants:
#define _REG_FIFO 0x00
#define _REG_OP_MODE 0x01
#define _REG_DATA_MOD 0x02
#define _REG_BITRATE_MSB 0x03
#define _REG_BITRATE_LSB 0x04
#define _REG_FDEV_MSB 0x05
#define _REG_FDEV_LSB 0x06
#define _REG_FRF_MSB 0x07
#define _REG_FRF_MID 0x08
#define _REG_FRF_LSB 0x09
#define _REG_VERSION 0x10
#define _REG_PA_LEVEL 0x11
#define _REG_RX_BW 0x19
#define _REG_AFC_BW 0x1A
#define _REG_RSSI_VALUE 0x24
#define _REG_DIO_MAPPING1 0x25
#define _REG_IRQ_FLAGS1 0x27
#define _REG_IRQ_FLAGS2 0x28
#define _REG_PREAMBLE_MSB 0x2C
#define _REG_PREAMBLE_LSB 0x2D
#define _REG_SYNC_CONFIG 0x2E
#define _REG_SYNC_VALUE1 0x2F
#define _REG_PACKET_CONFIG1 0x37
#define _REG_FIFO_THRESH 0x3C
#define _REG_PACKET_CONFIG2 0x3D
#define _REG_AES_KEY1 0x3E
#define _REG_TEMP1 0x4E
#define _REG_TEMP2 0x4F
#define _REG_TEST_PA1 0x5A
#define _REG_TEST_PA2 0x5C
#define _REG_TEST_DAGC 0x6F

#define _TEST_PA1_NORMAL 0x55
#define _TEST_PA1_BOOST 0x5D
#define _TEST_PA2_NORMAL 0x70
#define _TEST_PA2_BOOST 0x7C

//The crystal oscillator frequency and frequency synthesizer step size.
//See the datasheet for details of this calculation.

//        #define _FXOSC = 32000000.0
//        #define _FSTEP = _FXOSC / 524288

//RadioHead specific compatibility constants.
#define _RH_BROADCAST_ADDRESS 0x00
//The acknowledgement bit in the FLAGS
//The top 4 bits of the flags are reserved for RadioHead. The lower 4 bits are reserved
//for application layer use.
#define _RH_FLAGS_ACK 0x80
#define _RH_FLAGS_RETRY 0x40

//User facing constants:
#define SLEEP_MODE 0b000
#define STANDBY_MODE 0b001
#define FS_MODE 0b010
#define TX_MODE 0b011
#define RX_MODE 0b100
//supervisor.ticks_ms() contants
//          #define _TICKS_PERIOD = const(1 << 29)
//          #define _TICKS_MAX = const(_TICKS_PERIOD - 1)
//          #define _TICKS_HALFPERIOD = const(_TICKS_PERIOD // 2)

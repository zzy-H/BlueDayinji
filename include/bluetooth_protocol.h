#ifndef __BLUETOOTH_PROTOCOL_H
#define __BLUETOOTH_PROTOCOL_H

#include <Arduino.h>
#include <BluetoothSerial.h>
#include <main.h>

extern BluetoothSerial SerialBT;

#define START_BYTE  0x02
#define END_BYTE    0x03

#define PRINT_DATA           0
#define PRINT_DATA_COMPRESS  1
#define GET_VERSION          4
#define SENT_VERSION         5
#define GET_MODEL            6
#define SENT_MODEL           7
#define GET_BT_MAC           8
#define SENT_BT_MAC          9
#define GET_SN              10
#define SENT_SN             11
#define GET_STATUS          12
#define SENT_STATUS         13
#define GET_VOLTAGE         14
#define SENT_VOLTAGE        15
#define GET_BAT_STATUS      16
#define SENT_BAT_STATUS     17
#define GET_TEMP            18
#define SENT_TEMP           19
#define SET_HEAT_DENSITY    25
#define GET_HEAT_DENSITY    28
#define SENT_HEAT_DENSITY   29
#define SET_POWER_DOWN_TIME  30
#define GET_POWER_DOWN_TIME  31
#define SENT_POWER_DOWN_TIME 32
#define FEED_LINE           26
#define PRINT_TEST_PAGE     27
#define GET_PAPER_TYPE      42
#define SENT_PAPER_TYPE     43
#define SET_PAPER_TYPE      44
#define GET_COUNTRY_NAME    45
#define SENT_COUNTRY_NAME   46
#define GET_DEV_NAME        48
#define SENT_DEV_NAME       49
#define SET_CRC_KEY         24

#define CRC_KEY     0x35769521ul
#define DEV_SN      "P1001705253855"
#define DEV_VERSION { 0x01, 0x00, 0x02 }
#define DEV_MODEL   "BK3432"
#define DEV_NAME    "MaoPaperang"
#define COUNTRY_NAME    "CN"
#define CMD_66_DATA     "BK3432"
#define CMD_127_DATA    { 0x76, 0x33, 0x2e, 0x33, 0x38, 0x2e, 0x31, 0x39, 0x00, 0x00, 0x00, 0x00 }

typedef struct {
    uint8_t  packType;
    uint8_t  packIndex;
    uint16_t dataLen;
} PackHeader;

class CRC32_Class {
public:
    CRC32_Class() { init(0); }
    void init(uint32_t key);
    uint32_t calc(const uint8_t* data, uint32_t len);
private:
    uint32_t crc;
    static bool tableBuilt;
    static uint32_t table[256];
};

extern uint8_t gotStartByte;
extern uint8_t readpos;
extern PackHeader packHeader;
extern uint8_t dataPack_read[2048];
extern uint16_t dataPack_read_pos;
extern CRC32_Class crc32;

void paperang_send(void);
void paperang_send_ack(uint8_t type);
void paperang_send_msg(uint8_t type, const uint8_t* dat, uint16_t len);
void paperang_process_data(void);
void HeatTemp(void);

#endif

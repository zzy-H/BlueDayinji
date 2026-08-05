#include "bluetooth_protocol.h"
#include <motor.h>
#include <tph.h>
#include <print_test.h>
#include <main.h>

// ===== 协议全局变量 =====
uint8_t dataPack[520];
uint16_t dataPack_len;
uint8_t dataPack_read[2048];
uint16_t dataPack_read_pos;
uint8_t gotStartByte = 0;
uint8_t readpos = 0;
PackHeader packHeader;
CRC32_Class crc32;
uint8_t head_temp = 30;
uint16_t Finish_Out = 200;
uint8_t ongofront = 0;

// batVoltage, PRINTER_BATTERY, heat_density 声明在 motor.h

// ===== CRC32 =====
bool CRC32_Class::tableBuilt = false;
uint32_t CRC32_Class::table[256];

void CRC32_Class::init(uint32_t key) {
    crc = key;
    if (!tableBuilt) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t ch = i;
            for (int j = 0; j < 8; j++) {
                if (ch & 1) ch = 0xEDB88320 ^ (ch >> 1);
                else ch = ch >> 1;
            }
            table[i] = ch;
        }
        tableBuilt = true;
    }
}

uint32_t CRC32_Class::calc(const uint8_t* data, uint32_t len) {
    uint32_t c = crc ^ 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++)
        c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFF;
}

void paperang_send(void) { SerialBT.write(dataPack, dataPack_len); }

void paperang_send_ack(uint8_t type) {
    uint8_t ackcrc = 0;
    dataPack[0] = START_BYTE; dataPack[1] = type; dataPack[2] = 0x00;
    dataPack[3] = 0x01; dataPack[4] = 0x00; dataPack[5] = 0x00;
    uint32_t crc32_result = crc32.calc(&ackcrc, 1);
    memcpy(dataPack + 6, (uint8_t *)&crc32_result, 4);
    dataPack[10] = END_BYTE; dataPack_len = 11; paperang_send();
}

void paperang_send_msg(uint8_t type, const uint8_t* dat, uint16_t len) {
    dataPack[0] = START_BYTE; dataPack[1] = type; dataPack[2] = 0x00;
    memcpy(dataPack + 3, (uint8_t *)&len, 2);
    memcpy(dataPack + 5, dat, len);
    dataPack_len = 5 + len;
    uint32_t crc32_result = crc32.calc(dat, len);
    memcpy(dataPack + dataPack_len, (uint8_t *)&crc32_result, 4);
    dataPack[dataPack_len + 4] = END_BYTE;
    dataPack_len += 5; paperang_send();
}

void HeatTemp(void) { head_temp = 30; }

void paperang_process_data() {
    switch (packHeader.packType) {
        case PRINT_DATA: case PRINT_DATA_COMPRESS: return;
        case SET_CRC_KEY: {
            uint32_t tmp32 = (uint32_t)dataPack_read[0] | (uint32_t)dataPack_read[1]<<8 |
                             (uint32_t)dataPack_read[2]<<16 | (uint32_t)dataPack_read[3]<<24;
            crc32.init(tmp32); Serial.printf("CRC KEY: 0x%08X\r\n", tmp32); break;
        }
        case GET_SN: paperang_send_msg(SENT_SN, (uint8_t *)DEV_SN, strlen(DEV_SN)); break;
        case GET_VERSION: { uint8_t v[3]=DEV_VERSION; paperang_send_msg(SENT_VERSION,v,3); break; }
        case GET_MODEL: paperang_send_msg(SENT_MODEL,(uint8_t*)DEV_MODEL,strlen(DEV_MODEL)); break;
        case GET_STATUS: { uint8_t st=0; paperang_send_msg(SENT_STATUS,&st,1); break; }
        case GET_VOLTAGE: { BatteryPower(); uint16_t mv=(uint16_t)(batVoltage*1000); paperang_send_msg(SENT_VOLTAGE,(uint8_t*)&mv,2); break; }
        case GET_BAT_STATUS: { BatteryPower(); paperang_send_msg(SENT_BAT_STATUS,(uint8_t*)&PRINTER_BATTERY,1); break; }
        case GET_TEMP: HeatTemp(); paperang_send_msg(SENT_TEMP,&head_temp,1); break;
        case SET_HEAT_DENSITY: if(dataPack_read_pos>0){heat_density=dataPack_read[0]; Serial.printf("HEAT:%d\n",heat_density);} break;
        case GET_HEAT_DENSITY: paperang_send_msg(SENT_HEAT_DENSITY,&heat_density,1); break;
        case FEED_LINE: if(printDataCount/48!=0){startPrint();ongofront=1;goFront(Finish_Out,800);ongofront=0;} printDataCount=0; break;
        case GET_POWER_DOWN_TIME: { uint16_t pdt=3600; paperang_send_msg(SENT_POWER_DOWN_TIME,(uint8_t*)&pdt,2); break; }
        case GET_PAPER_TYPE: { uint8_t pt=0; paperang_send_msg(SENT_PAPER_TYPE,&pt,1); break; }
        case PRINT_TEST_PAGE: Print_test(); break;
        case SET_PAPER_TYPE: break;
        case GET_COUNTRY_NAME: paperang_send_msg(SENT_COUNTRY_NAME,(uint8_t*)COUNTRY_NAME,2); break;
        case GET_DEV_NAME: paperang_send_msg(SENT_DEV_NAME,(uint8_t*)DEV_NAME,strlen(DEV_NAME)); break;
        case 66: paperang_send_msg(67,(uint8_t*)CMD_66_DATA,strlen(CMD_66_DATA)); break;
        case 127: { uint8_t d127[12]=CMD_127_DATA; paperang_send_msg(128,d127,12); break; }
        default: Serial.printf("UNKNOWN:%d\n",packHeader.packType); break;
    }
    paperang_send_ack(packHeader.packType);
}

#include "bluetooth_protocol.h"
#include <motor.h>
#include <tph.h>
#include <print_test.h>
#include <main.h>

/* ==================================================================
 * 喵喵机（Paperang）蓝牙协议层（bluetooth_protocol.cpp）
 *
 * 职责：
 *   1. 解析 APP 下发的各类命令/查询（版本/型号/状态/电量/加热浓度...）
 *   2. 按协议格式打包回复报文（帧头 + 类型 + 长度 + 数据 + CRC32 + 帧尾）
 *   3. CRC32 校验（查表法，标准 CRC-32 多项式 0xEDB88320）
 * ================================================================== */

// ===== 协议全局变量 =====
uint8_t dataPack[520];           // 发送缓冲（打包回复报文用）
uint16_t dataPack_len;           // 发送缓冲长度
uint8_t dataPack_read[2048];     // 接收缓冲（APP 下发的命令/参数数据）
uint16_t dataPack_read_pos;      // 接收缓冲写入位置
uint8_t gotStartByte = 0;        // 是否已收帧头
uint8_t readpos = 0;             // 解析位置
PackHeader packHeader;           // 帧头信息
CRC32_Class crc32;               // CRC32 对象
uint8_t head_temp = 30;          // 打印头温度（模拟值 30°C，APP 查询用）
uint16_t Finish_Out = 200;       // 打完后的送纸长度（步数）
uint8_t ongofront = 0;           // 走纸标志

// batVoltage, PRINTER_BATTERY, heat_density 声明在 motor.h（由 print_test.cpp 定义）

// ===== CRC32 实现（查表法） =====
bool CRC32_Class::tableBuilt = false;    // 查表是否已生成（静态，只生成一次）
uint32_t CRC32_Class::table[256];        // CRC32 查表（静态存储）

/* 初始化：设置初始密钥 + 生成 256 项查表（只生成一次） */
void CRC32_Class::init(uint32_t key) {
    crc = key;
    if (!tableBuilt) {
        // 标准 CRC-32 查表生成（多项式 0xEDB88320，反射算法）
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

/* 计算 CRC32：逐字节查表更新
 * 标准流程：初始值取反 → 逐字节异或查表 → 结果取反 */
uint32_t CRC32_Class::calc(const uint8_t* data, uint32_t len) {
    uint32_t c = crc ^ 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++)
        c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFF;
}

/* 发送数据包（写蓝牙串口） */
void paperang_send(void) { SerialBT.write(dataPack, dataPack_len); }

/* 发送简单 ACK 应答（11 字节）
 * 格式：START + type + 0x00 + 0x01,0x00(长度=1) + 1字节数据(0x00) + CRC32 + END */
void paperang_send_ack(uint8_t type) {
    uint8_t ackcrc = 0;
    dataPack[0] = START_BYTE; dataPack[1] = type; dataPack[2] = 0x00;
    dataPack[3] = 0x01; dataPack[4] = 0x00; dataPack[5] = 0x00;   // 长度1 + 1字节数据
    uint32_t crc32_result = crc32.calc(&ackcrc, 1);   // 对 1 字节数据算 CRC
    memcpy(dataPack + 6, (uint8_t *)&crc32_result, 4);  // 写 CRC（4 字节小端）
    dataPack[10] = END_BYTE; dataPack_len = 11; paperang_send();
}

/* 发送带数据的回复报文
 * 格式：START + type + 0x00 + len(2字节小端) + dat + CRC32(4) + END */
void paperang_send_msg(uint8_t type, const uint8_t* dat, uint16_t len) {
    dataPack[0] = START_BYTE; dataPack[1] = type; dataPack[2] = 0x00;
    memcpy(dataPack + 3, (uint8_t *)&len, 2);      // 数据长度（小端）
    memcpy(dataPack + 5, dat, len);                // 数据内容
    dataPack_len = 5 + len;
    uint32_t crc32_result = crc32.calc(dat, len);  // 对数据算 CRC
    memcpy(dataPack + dataPack_len, (uint8_t *)&crc32_result, 4);  // 写 CRC
    dataPack[dataPack_len + 4] = END_BYTE;
    dataPack_len += 5; paperang_send();
}

/* 温度上报辅助（固定返回 30°C，未接温度传感器） */
void HeatTemp(void) { head_temp = 30; }

/* ==================================================================
 * 处理一帧完整的蓝牙数据（收到 END_BYTE 后由 main.cpp 调用）
 *
 * 根据 packType 分发：
 *   - PRINT_DATA/PRINT_DATA_COMPRESS：打印数据，不在这里处理（数据已存 printData）
 *   - 查询类：打包回复（版本/型号/SN/状态/电量/温度...）
 *   - 设置类：改参数（CRC密钥/加热浓度）
 *   - 动作类：打印测试页/送纸
 * 处理完统一发 ACK
 * ================================================================== */
void paperang_process_data() {
    switch (packHeader.packType) {
        case PRINT_DATA: case PRINT_DATA_COMPRESS: return;  // 打印数据已在 main.cpp 存缓冲，这里跳过

        /* ---- 设置类 ---- */
        case SET_CRC_KEY: {
            // 从数据区读 4 字节小端 → 重新初始化 CRC 密钥
            uint32_t tmp32 = (uint32_t)dataPack_read[0] | (uint32_t)dataPack_read[1]<<8 |
                             (uint32_t)dataPack_read[2]<<16 | (uint32_t)dataPack_read[3]<<24;
            crc32.init(tmp32); Serial.printf("CRC KEY: 0x%08X\r\n", tmp32); break;
        }
        case SET_HEAT_DENSITY:
            // APP 设置加热浓度（第 1 字节）
            if(dataPack_read_pos>0){heat_density=dataPack_read[0]; Serial.printf("HEAT:%d\n",heat_density);} break;
        case SET_PAPER_TYPE: break;   // 纸张类型设置（暂不处理）

        /* ---- 查询类：打包回复 ---- */
        case GET_SN: paperang_send_msg(SENT_SN, (uint8_t *)DEV_SN, strlen(DEV_SN)); break;           // 序列号
        case GET_VERSION: { uint8_t v[3]=DEV_VERSION; paperang_send_msg(SENT_VERSION,v,3); break; }  // 固件版本
        case GET_MODEL: paperang_send_msg(SENT_MODEL,(uint8_t*)DEV_MODEL,strlen(DEV_MODEL)); break;  // 型号
        case GET_STATUS: { uint8_t st=0; paperang_send_msg(SENT_STATUS,&st,1); break; }              // 状态
        case GET_VOLTAGE: { BatteryPower(); uint16_t mv=(uint16_t)(batVoltage*1000); paperang_send_msg(SENT_VOLTAGE,(uint8_t*)&mv,2); break; }  // 电压(mV)
        case GET_BAT_STATUS: { BatteryPower(); paperang_send_msg(SENT_BAT_STATUS,(uint8_t*)&PRINTER_BATTERY,1); break; }  // 电量百分比
        case GET_TEMP: HeatTemp(); paperang_send_msg(SENT_TEMP,&head_temp,1); break;                 // 打印头温度
        case GET_HEAT_DENSITY: paperang_send_msg(SENT_HEAT_DENSITY,&heat_density,1); break;          // 当前浓度
        case GET_POWER_DOWN_TIME: { uint16_t pdt=3600; paperang_send_msg(SENT_POWER_DOWN_TIME,(uint8_t*)&pdt,2); break; }  // 自动关机时间
        case GET_PAPER_TYPE: { uint8_t pt=0; paperang_send_msg(SENT_PAPER_TYPE,&pt,1); break; }      // 纸张类型
        case GET_COUNTRY_NAME: paperang_send_msg(SENT_COUNTRY_NAME,(uint8_t*)COUNTRY_NAME,2); break; // 国家代码
        case GET_DEV_NAME: paperang_send_msg(SENT_DEV_NAME,(uint8_t*)DEV_NAME,strlen(DEV_NAME)); break;  // 设备名

        /* ---- 动作类 ---- */
        case FEED_LINE:
            // 送纸指令：先打已有数据，再走纸 Finish_Out 步，清缓冲
            if(printDataCount/48!=0){startPrint();ongofront=1;goFront(Finish_Out,800);ongofront=0;}
            printDataCount=0; break;
        case PRINT_TEST_PAGE: Print_test(); break;   // 打印测试页

        /* ---- 兼容未知命令 ---- */
        case 66: paperang_send_msg(67,(uint8_t*)CMD_66_DATA,strlen(CMD_66_DATA)); break;
        case 127: { uint8_t d127[12]=CMD_127_DATA; paperang_send_msg(128,d127,12); break; }

        default: Serial.printf("UNKNOWN:%d\n",packHeader.packType); break;
    }
    paperang_send_ack(packHeader.packType);   // 统一回 ACK
}

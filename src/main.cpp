#include <Arduino.h>
#include <main.h>
#include <led.h>
#include <motor.h>
#include <tph.h>
#include <print_test.h>
#include "bluetooth_protocol.h"

BluetoothSerial SerialBT;   // ESP32 经典蓝牙串口对象

// 从 bluetooth_protocol.cpp 引用的全局变量
extern uint8_t gotStartByte;      // 是否已收到帧头（0x02）
extern uint8_t readpos;           // 当前解析到帧的第几个字节
extern PackHeader packHeader;     // 帧头信息（类型/序号/长度）
extern uint8_t dataPack_read[];   // 非打印数据的接收缓冲（命令/参数）
extern uint16_t dataPack_read_pos;// 该缓冲写入位置
extern CRC32_Class crc32;         // CRC32 校验对象

static uint16_t recv_count = 0;   // 已接收的数据字节数（对照 dataLen）
static uint16_t recv_total = 0;   // 本帧应接收的总数据字节数

void setup() {
    Led_Config();          // LED 引脚初始化
    Printer_Config();      // 打印头 + 电机引脚初始化
    Battery_Config();      // 电池 ADC 引脚配置
    Serial.begin(115200);  // 调试串口

    // 初始化 HSPI 外设（CLK=15, MOSI=13；MISO 未用传 -1）
    printerSPI.begin(PIN_CLK, PIN_MISO, PIN_MOSI, -1);

    crc32.init(CRC_KEY);                 // 初始化 CRC32（默认密钥 0x35769521）
    SerialBT.begin("MaoPaperang");       // 开启蓝牙，广播名"MaoPaperang"（喵喵机 APP 识别）
    delay(100);
    Serial.println("INIT OK");
}

void loop() {
    /* ============ 蓝牙数据接收：逐字节状态机 ============
     * 协议帧格式：
     *   [0]   START_BYTE(0x02)  帧头
     *   [1]   packType          包类型（打印数据/命令/查询...）
     *   [2]   packIndex         包序号
     *   [3:4] dataLen           数据长度（小端，uint16）
     *   [5..] data              数据内容
     *   [..]  CRC32(4字节)      数据校验
     *   [..]  END_BYTE(0x03)    帧尾
     *
     * 状态机设计：不阻塞、逐字节推进，UART FIFO 满也不会丢数据 */
    while (SerialBT.available()) {
        uint8_t c = SerialBT.read();
        PowerONTime = millis();          // 收到数据 = 设备活动，刷新时间

        if (!gotStartByte) {
            // 还没见到帧头：只找 0x02
            if (c == START_BYTE) { gotStartByte=1; readpos=0; packHeader.dataLen=0; }
            continue;
        }

        // 已过帧头：按 readpos 解析各个字段
        switch (readpos) {
            case 0: packHeader.packType=c; readpos=1; break;   // 包类型
            case 1: packHeader.packIndex=c; readpos=2; break;  // 包序号
            case 2: packHeader.dataLen=c; readpos=3; break;    // 长度低字节
            case 3:
                // 长度高字节（小端拼接）
                packHeader.dataLen |= (uint16_t)c << 8;
                // 数据长度 0 或 ≥2048 非法 → 跳到帧尾等待结束
                if (packHeader.dataLen==0||packHeader.dataLen>=2048) { readpos=8; }
                else { recv_count=0; recv_total=packHeader.dataLen; dataPack_read_pos=0; readpos=4; }
                break;
            case 4:
                // 数据阶段：按类型分流
                if (packHeader.packType==PRINT_DATA||packHeader.packType==PRINT_DATA_COMPRESS) {
                    // 打印数据 → 存到 printData（大缓冲），打满为止
                    if (printDataCount<PRINT_DATA_MAX) printData[printDataCount++]=c;
                } else {
                    // 命令/参数数据 → 存到 dataPack_read（2048 上限）
                    if (dataPack_read_pos<2048) dataPack_read[dataPack_read_pos++]=c;
                }
                recv_count++;
                if (recv_count>=recv_total) readpos=8;   // 数据收完 → 跳帧尾
                break;
            default:
                // 帧尾阶段（CRC 4 字节 + END_BYTE）：readpos 8~12
                readpos++;
                if (readpos>=13) {
                    // 收到帧尾 → 整帧完成 → 处理
                    if(c==END_BYTE) paperang_process_data();
                    gotStartByte=0; readpos=0;   // 复位状态机，等下一帧
                }
                break;
        }
    }

    /* ============ 蓝牙连接状态监测 ============ */
    static bool was=false;
    bool now=SerialBT.hasClient();
    if(now!=was){was=now; if(now) Serial.println("BT CONNECTED!");}  // 刚连上打印提示

    /* ============ 电池电量周期上报 ============ */
    static uint32_t last_bat=0;
    if(millis()-last_bat>5000){last_bat=millis(); BatteryPower();}   // 每 5 秒测一次

    /* ============ 按键测试打印 ============ */
    static uint8_t btn=0;
    if(digitalRead(BUTTON)==0){                    // 按键按下（低电平）
        if(!btn){                                  // 去抖：只在边沿触发一次
            delay(30);                             // 简单软件去抖
            if(digitalRead(BUTTON)==0){
                btn=1;LED_ON();Print_test();LED_OFF();   // 打印本地测试图片
            }
        }
    }
    else btn=0;   // 松开复位标志

    delay(1);
}

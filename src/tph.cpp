#include <motor.h>
#include <tph.h>

/* ==================================================================
 * 热敏打印头驱动（tph.cpp）
 *
 * 核心职责：
 *   1. 配置打印头相关引脚（SPI 数据线 + 6 路 STB 选通 + 加热电源）
 *   2. 把图像数据分批送入打印头并加热（分批加热算法，防过流）
 *   3. 配合步进电机逐行走纸
 *
 * 打印头工作流程（每行）：
 *   ① SPI 发送 48 字节（384 位）→ 打印头移位寄存器
 *   ② LAT 上升沿 → 数据锁存到锁存器
 *   ③ 6 个 STB 同时拉高 → 发热点通电 → 热敏纸变黑
 *   ④ 延时加热时间后 STB 拉低
 *   ⑤ 步进电机走 4 步 → 移到下一行
 * ================================================================== */

// 使用 ESP32 的 HSPI 外设驱动打印头（不占用 Flash 引脚！GPIO6 是 Flash 时钟，不能用）
SPIClass printerSPI(HSPI);
// SPI 配置：1MHz，高位在前，模式 0（空闲低电平，第一个边沿采样）
SPISettings printerSPISettings = SPISettings(1000000, SPI_MSBFIRST, SPI_MODE0);

uint8_t motorPos = 0;          // 当前电机节拍位置（0~7，8 拍半步进）
volatile uint8_t onprint = 0;  // 打印中标志（1=正在打印，供协议层判断是否可收新数据）
volatile uint8_t PaperSta = 1; // 缺纸检测状态（1=有纸；当前未接硬件中断，保留扩展）
uint32_t PowerONTime = 0;      // 最近一次活动时间（蓝牙收到数据/打印时刷新，用于超时/省电判断）

uint8_t printData[PRINT_DATA_MAX] __attribute__((aligned(4))) = {0};  // 整幅图像数据缓冲（4字节对齐，SPI DMA 友好）
uint16_t printDataCount = 0;   // 已收到的图像数据字节数（蓝牙一帧帧累加）
uint8_t printDatacache[48] = {0};  // 单行打印缓存（48 字节 = 384 个发热点）
uint8_t heat_density = 80;     // 加热浓度（%）：APP 可调（50~100），决定打印深浅
uint16_t printpin = 24;        // 每批最多加热的点数（分批加热限制，防瞬间过流烧电源）

/* 查表法统计一个字节中"1"的个数（即发热点数）
 * 例如 shuzu16to2jingzhi[0x0F] = 4（二进制 00001111 有 4 个 1）
 * 用 256 项查表代替逐位循环，速度快（每行要统计 48 次） */
const uint8_t shuzu16to2jingzhi[256] = {
    0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,5,6,6,7,6,7,7,8
};

/* 8 拍半步进电机节拍表
 * 每拍只改变一个线圈状态 → 运行平稳、力矩均匀、无共振
 * 每拍：[AP, AM, BP, BM]（1=导通，0=截止）
 * 遍历方向：motorPos 递减 = 反向遍历 = 出纸（问题3 修复） */
static const uint8_t motorTable[8][4] = {
    {1,0,0,0},{1,0,1,0},{0,0,1,0},{0,1,1,0},
    {0,1,0,0},{0,1,0,1},{0,0,0,1},{1,0,0,1}
};

/* 关闭所有 6 路 STB 选通（停止加热） */
void clearSTB(void) {
    digitalWrite(PIN_STB1,0); digitalWrite(PIN_STB2,0);
    digitalWrite(PIN_STB3,0); digitalWrite(PIN_STB4,0);
    digitalWrite(PIN_STB5,0); digitalWrite(PIN_STB6,0);
}

/* 清空单行打印缓存（48 字节全清 0 → 全部不发热） */
void clearData(void) {
    for(int i=0;i<48;i++) printDatacache[i]=0;
}

/* 打印头 + 电机引脚初始化 */
void Printer_Config(void) {
    pinMode(PIN_MOTOR_AP, OUTPUT);   // 电机 A 相 +
    pinMode(PIN_MOTOR_AM, OUTPUT);   // 电机 A 相 -
    pinMode(PIN_MOTOR_BP, OUTPUT);   // 电机 B 相 +
    pinMode(PIN_MOTOR_BM, OUTPUT);   // 电机 B 相 -
    pinMode(PIN_VHEN, OUTPUT);       // 加热电源使能（升压模块 EN 脚）
    pinMode(PIN_STB1, OUTPUT);       // 选通段 1~6（每段控制 64 个发热点）
    pinMode(PIN_STB2, OUTPUT);
    pinMode(PIN_STB3, OUTPUT);
    pinMode(PIN_STB4, OUTPUT);
    pinMode(PIN_STB5, OUTPUT);
    pinMode(PIN_STB6, OUTPUT);
    // 注意：PIN_CLK(15)/PIN_MOSI(13) 不需要 pinMode —— 它们由 HSPI 外设接管！
    pinMode(PIN_LAT, OUTPUT);        // 锁存脚（数据送完后拉上升沿锁存）
    pinMode(PIN_PTEST, INPUT);       // 缺纸检测（当前保留未用）

    digitalWrite(PIN_LAT,1);         // LAT 默认高（SPI 送完后 拉低→拉高 产生上升沿）
    digitalWrite(PIN_VHEN,0);        // 先关闭加热电源（上电安全）
    digitalWrite(PIN_MOTOR_AP,0);    // 电机初始全部截止（不转）
    digitalWrite(PIN_MOTOR_AM,0);
    digitalWrite(PIN_MOTOR_BP,0);
    digitalWrite(PIN_MOTOR_BM,0);
    clearSTB();                      // 关闭所有选通（不加热）
}

/* 步进电机走 1 拍（正转 / 出纸方向）
 * 从节拍表取出当前拍 → 写入 4 个电机引脚 → 节拍指针递减
 * 注意：motorPos-- 实现"出纸"方向（问题3：递增会导致纸往里卷） */
void goFront1(uint16_t wait) {
    digitalWrite(PIN_MOTOR_AP, motorTable[motorPos][0]);
    digitalWrite(PIN_MOTOR_AM, motorTable[motorPos][1]);
    digitalWrite(PIN_MOTOR_BP, motorTable[motorPos][2]);
    digitalWrite(PIN_MOTOR_BM, motorTable[motorPos][3]);
    if(motorPos==0) motorPos=8;   // 回绕到表尾（防止下溢）
    --motorPos;                    // 递减 = 反向遍历节拍表 = 出纸
    if(wait>0) delayMicroseconds(wait);  // 步进间隔（控制转速，单位 µs）
}

/* 步进电机走 N 拍 */
void goFront(uint16_t steps, uint16_t wait) {
    for(uint16_t i=0;i<steps;i++) goFront1(wait);
}

/* 通过 HSPI 发送一行数据（48 字节）到打印头，然后锁存
 * SPI 时序：发 384 位 → 移位寄存器满 → LAT 上升沿 → 锁存到锁存器 */
void sendData(uint8_t *data) {
    printerSPI.beginTransaction(printerSPISettings);   // 开始 SPI 事务（应用 1MHz 配置）
    for(uint8_t i=0;i<48;i++) {
        printerSPI.transfer(data[i]);   // 逐字节发到移位寄存器（48×8=384 位）
    }
    printerSPI.endTransaction();        // 结束 SPI 事务

    digitalWrite(PIN_LAT, 0);           // LAT 拉低
    delayMicroseconds(1);               // 稳定 1µs
    digitalWrite(PIN_LAT, 1);           // 拉高 → 上升沿 → 数据锁存
}

/* ==================================================================
 * 打印整幅图像（核心函数）
 *
 * 分批加热原理：
 *   一行 48 字节可能有很多"1"（发热点），如果全部同时加热，
 *   瞬间电流可达 5A+，会烧坏电源或打印头。
 *   所以把一行拆成多批：每批最多 printpin(24) 个发热点。
 *
 *   分批策略（cache 机制）：
 *     从第 0 个字节开始累加发热点数，没超过 24 就加入本批；
 *     一旦超过 24，把"当前字节"作为下一批的起点，本批先加热。
 *     这样每批 ≤ 24 点，电流可控；多批合起来完成一行。
 * ================================================================== */
void startPrint(void) {
    onprint=1;                     // 置打印标志（协议层据此判断是否在打印）
    PowerONTime=millis();          // 刷新活动时间
    Serial.printf("[INFO] 打印 %u 行\n", printDataCount/48);   // 行数 = 总字节数/48

    digitalWrite(PIN_VHEN,1);      // 开启加热电源（升压模块工作）
    delayMicroseconds(5000);       // 等电源稳定 5ms

    // 逐行处理：每行 48 字节
    for(uint32_t pointer=0;pointer<printDataCount;pointer+=48) {
        int cache=-1;              // cache = 本批处理的起始字节索引（-1 = 刚开始）
        while(cache!=0) {          // cache 回到 0 = 本行处理完
            if(cache==-1) cache=0; // 第一次进来从字节 0 开始
            int pinnumber=0;       // 本批累计发热点数
            int i;
            // 从 cache 位置开始累加发热点，直到超过 printpin(24)
            for(i=cache;i<48;i++) {
                pinnumber+=shuzu16to2jingzhi[printData[pointer+i]];  // 查表统计本字节发热点数
                if(pinnumber<=printpin) {
                    printDatacache[i]=printData[pointer+i];   // 没超限：本字节加入本批
                    if(i==47) cache=0;                        // 到行尾：本批就是最后一批
                } else { cache=i; break; }                    // 超限：从本字节开始下一批
            }
            sendData(printDatacache);   // 送本批数据 → SPI → 锁存
            clearData();                // 清缓存准备下一批

            if(pinnumber!=0) {          // 本批有发热点才加热（全 0 行跳过，省时间）
                // 6 路 STB 全部拉高 → 发热点通电加热
                digitalWrite(PIN_STB1,1); digitalWrite(PIN_STB2,1);
                digitalWrite(PIN_STB3,1); digitalWrite(PIN_STB4,1);
                digitalWrite(PIN_STB5,1); digitalWrite(PIN_STB6,1);
                // 加热时间 = 基础时间 × 浓度/100（APP 调 heat_density 控制深浅）
                delayMicroseconds(PRINT_TIME * heat_density / 100);
                clearSTB();             // 关 STB 停止加热
                delayMicroseconds(PRINT_TIME_);   // 加热后冷却间隔
            }
        }
        // 一行打完 → 走纸 4 步到下一行（3 步正常 + 1 步保持，防行缝隙/重叠）
        goFront1(800); goFront1(800); goFront1(800); goFront1(0);
        yield();   // 让出 CPU（长打印期间保持系统响应，ESP32 看门狗不炸）
    }

    // 打印结束：电机停、关加热、清缓冲、关电源
    digitalWrite(PIN_MOTOR_AP,0); digitalWrite(PIN_MOTOR_AM,0);
    digitalWrite(PIN_MOTOR_BP,0); digitalWrite(PIN_MOTOR_BM,0);
    clearSTB(); clearData();
    printDataCount=0;              // 清空图像数据（为下一张图做准备）
    digitalWrite(PIN_VHEN,0);      // 关闭加热电源（省电 + 安全）
    Serial.println("[INFO] 打印完成");
    onprint=0;                     // 清打印标志
}

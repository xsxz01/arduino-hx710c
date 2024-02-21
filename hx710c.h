#pragma once
#define HX710C_LIB_VERSION (F("0.0.1"))
#define HX710C_DEFAULT_CLOCK_FFREQUENCY_10HZ (10)
#define HX710C_CLOCK_FFREQUENCY_40HZ (40)
/// @brief read mode
enum HX710C_MODE
{
    HX710C_MODE_AVERAGE = 0x00,
    HX710C_MODE_MEDIAN,
    HX710C_MODE_MEDAVG,
    HX710C_MODE_RUNAVG,
    HX710C_MODE_RAW
};
#include "Arduino.h"

class hx710c
{
private:
    uint8_t _dataPin;
    uint8_t _clockPin;

    /// @brief 时钟频率
    uint8_t _clockFrequency = HX710C_DEFAULT_CLOCK_FFREQUENCY_10HZ; //  default channel A
    long _offset = 0;
    float _scale = 1;
    uint32_t _lastRead = 0;
    float _price = 0;
    uint8_t _mode = 0;

    void _insertSort(float *array, uint8_t size);
    uint8_t _shiftIn();

public:
    hx710c(/* args */);
    ~hx710c();

    void reset();
    /// @brief 开始读取
    /// @param dataPin DOUT
    /// @param clockPin PD_SCK
    void begin(uint8_t dataPin, uint8_t clockPin);

    //  检测是否已经准备好读数据
    bool is_ready();

    //  等待直到数据准备好
    //  每毫秒检测
    void wait_ready(uint32_t ms = 0);
    //  最大尝试次数
    bool wait_ready_retry(uint8_t retries = 3, uint32_t ms = 0);
    //  最大超时
    bool wait_ready_timeout(uint32_t timeout = 1000, uint32_t ms = 0);

    //  读原始数据
    float read();

    //  取平均数
    //  times = 1 or more
    float read_average(uint8_t times = 10);

    //  取中位数
    //  times = 3..15 - 奇数
    float read_median(uint8_t times = 7);

    //  取中位平均值
    //  times = 3..15 - 奇数
    float read_medavg(uint8_t times = 7);

    //  使用 running average 算法取值
    //  the weight alpha can be set to any value between 0 and 1
    //  times = 1 or more.
    float read_runavg(uint8_t times = 7, float alpha = 0.5);

    // 设置get_value()的取值方式，中位数和中为平均数只允许3到15
    void set_raw_mode();
    void set_average_mode();
    void set_median_mode();
    void set_medavg_mode();
    //  set_run_avg will use a default alpha of 0.5.
    void set_runavg_mode();
    uint8_t get_mode();

    //  corrected for offset.
    //  in HX710C_RAW_MODE the parameter times will be ignored.
    float get_value(uint8_t times = 1);
    //  converted to proper units, corrected for scale.
    //  in HX710C_RAW_MODE the parameter times will be ignored.
    float get_units(uint8_t times = 1);

    //  去皮
    //  call tare to calibrate zero
    void tare(uint8_t times = 10);
    float get_tare();
    bool tare_set();

    // 这里是设置芯片频率
    bool set_clock_frequency(uint8_t clock_frequency = HX710C_DEFAULT_CLOCK_FFREQUENCY_10HZ, bool forced = false);
    uint8_t get_clock_frequency();

    //  标定
    //  SCALE > 0
    //  returns false if scale == 0;
    bool set_scale(float scale = 1.0);
    float get_scale();

    //  OFFSET > 0
    void set_offset(long offset = 0);
    long get_offset();

    //  清除scale
    //  调用去皮来设置零点
    //  在磅秤上加一个已知的重量
    //  调用 calibrate_scale(weight)
    //  计算得到scale
    void calibrate_scale(uint16_t weight, uint8_t times = 10);

    //  芯片供电
    void power_down();
    void power_up();

    //  上次读的数据
    uint32_t last_read();

    //  计价
    float get_price(uint8_t times = 1) { return get_units(times) * _price; };
    void set_unit_price(float price = 1.0) { _price = price; };
    float get_unit_price() { return _price; };
};

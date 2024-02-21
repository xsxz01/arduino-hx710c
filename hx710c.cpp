#include "hx710c.h"

void hx710c::_insertSort(float *array, uint8_t size)
{
    // 插入排序
    uint8_t t, z;
    float temp;
    for (t = 1; t < size; t++)
    {
        z = t;
        temp = array[z];
        while ((z > 0) && (temp < array[z - 1]))
        {
            array[z] = array[z - 1];
            z--;
        }
        array[z] = temp;
        yield();
    }
}

uint8_t hx710c::_shiftIn()
{
    // 为了更快，选择使用局部变量
    uint8_t clk = _clockPin;
    uint8_t data = _dataPin;
    uint8_t value = 0;
    uint8_t mask = 0x80;
    while (mask > 0)
    {
        digitalWrite(clk, HIGH);
        delayMicroseconds(1);
        if (digitalRead(data) == HIGH)
        {
            value |= mask;
        }
        digitalWrite(clk, LOW);
        delayMicroseconds(1);
        mask >>= 1;
    }

    return value;
}

hx710c::hx710c(/* args */)
{
    reset();
}

hx710c::~hx710c()
{
}

void hx710c::reset()
{
    power_down();
    power_up();
    _offset = 0;
    _scale = 1;
    _clockFrequency = HX710C_DEFAULT_CLOCK_FFREQUENCY_10HZ;
    _mode = HX710C_MODE::HX710C_MODE_RAW;
    _lastRead = 0;
}

void hx710c::begin(uint8_t dataPin, uint8_t clockPin)
{
    _dataPin = dataPin;
    _clockPin = clockPin;

    pinMode(_dataPin, INPUT);
    pinMode(_clockPin, OUTPUT);
    digitalWrite(_clockPin, LOW);

    reset();
}

bool hx710c::is_ready()
{
    return digitalRead(_dataPin) == LOW;
}

void hx710c::wait_ready(uint32_t ms)
{
    while (!is_ready())
    {
        delay(ms);
    }
}

bool hx710c::wait_ready_retry(uint8_t retries, uint32_t ms)
{
    while (retries-- > 0)
    {
        if (is_ready())
            return true;
        delay(ms);
    }

    return false;
}

bool hx710c::wait_ready_timeout(uint32_t timeout, uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < timeout)
    {
        if (is_ready())
            return true;
        delay(ms);
    }
    return false;
}

float hx710c::read()
{
    // 如果DOUT是低电平则继续操作
    while (digitalRead(_dataPin) == HIGH)
    {
        yield();
    }
    // 应该是存放读取到的数据
    union hx710c_data
    {
        long value = 0;
        uint8_t data[4];
    } v;
    // 禁用中断
    noInterrupts();
    // 发出24个时钟脉冲读取数据
    v.data[2] = _shiftIn();
    v.data[1] = _shiftIn();
    v.data[0] = _shiftIn();

    // 10Hz
    uint8_t m = 1;
    if (_clockFrequency == HX710C_DEFAULT_CLOCK_FFREQUENCY_10HZ)
    {
        m = 1;
    }
    else if (_clockFrequency == HX710C_CLOCK_FFREQUENCY_40HZ)
    {
        m = 3;
    }

    // 写时钟脉冲
    while (m > 0)
    {
        digitalWrite(_clockPin, HIGH);
        digitalWrite(_clockPin, LOW);
        m--;
    }
    // 启用中断
    interrupts();
    if (v.data[2] & 0x80)
    {
        v.data[3] = 0xFF;
    }
    _lastRead = millis();
    return 1.0f * v.value;
}

float hx710c::read_average(uint8_t times)
{
    if (times < 1)
        times = 1;
    float sum = 0;
    for (uint8_t i = 0; i < times; i++)
    {
        sum += read();
        yield();
    }
    return sum / times;
}

float hx710c::read_median(uint8_t times)
{
    if (times > 15)
        times = 15;
    if (times < 3)
        times = 3;
    float samples[15];
    for (uint8_t i = 0; i < times; i++)
    {
        samples[i] = read();
        yield();
    }
    _insertSort(samples, times);
    if (times & 0x01)
        return samples[times / 2];
    return (samples[times / 2] + samples[times / 2 + 1]) / 2;
}

float hx710c::read_medavg(uint8_t times)
{
    if (times > 15)
        times = 15;
    if (times < 3)
        times = 3;
    float samples[15];
    for (uint8_t i = 0; i < times; i++)
    {
        samples[i] = read();
        yield();
    }
    _insertSort(samples, times);
    float sum = 0;
    //  iterate over 1/4 to 3/4 of the array
    uint8_t count = 0;
    uint8_t first = (times + 2) / 4;
    uint8_t last = times - first - 1;
    for (uint8_t i = first; i <= last; i++) //  !! include last one too
    {
        sum += samples[i];
        count++;
    }
    return sum / count;
}

float hx710c::read_runavg(uint8_t times, float alpha)
{
    if (times < 1)
        times = 1;
    if (alpha < 0)
        alpha = 0;
    if (alpha > 1)
        alpha = 1;
    float val = read();
    for (uint8_t i = 1; i < times; i++)
    {
        val += alpha * (read() - val);
        yield();
    }
    return val;
}

void hx710c::set_raw_mode()
{
    _mode = HX710C_MODE::HX710C_MODE_RAW;
}

void hx710c::set_average_mode()
{
    _mode = HX710C_MODE::HX710C_MODE_AVERAGE;
}

void hx710c::set_median_mode()
{
    _mode = HX710C_MODE::HX710C_MODE_MEDIAN;
}

void hx710c::set_medavg_mode()
{
    _mode = HX710C_MODE::HX710C_MODE_MEDAVG;
}

void hx710c::set_runavg_mode()
{
    _mode = HX710C_MODE::HX710C_MODE_RUNAVG;
}

uint8_t hx710c::get_mode()
{
    return _mode;
}

float hx710c::get_value(uint8_t times)
{
    float raw;
    switch (_mode)
    {
    case HX710C_MODE::HX710C_MODE_RAW:
        raw = read();
        break;
    case HX710C_MODE::HX710C_MODE_RUNAVG:
        raw = read_runavg(times);
        break;
    case HX710C_MODE::HX710C_MODE_MEDAVG:
        raw = read_medavg(times);
        break;
    case HX710C_MODE::HX710C_MODE_MEDIAN:
        raw = read_median(times);
        break;
    case HX710C_MODE::HX710C_MODE_AVERAGE:
    default:
        raw = read_average(times);
        break;
    }
    return raw - _offset;
}

float hx710c::get_units(uint8_t times)
{
    float units = get_value(times) * _scale;
    return units;
}

void hx710c::tare(uint8_t times)
{
    _offset = get_value(times);
}

float hx710c::get_tare()
{
    return -_offset * _scale;
}

bool hx710c::tare_set()
{
    return _offset != 0;
}

bool hx710c::set_clock_frequency(uint8_t clock_frequency, bool forced)
{
    if ((not forced) && (_clockFrequency == clock_frequency))
        return true;
    switch (clock_frequency)
    {
    case HX710C_DEFAULT_CLOCK_FFREQUENCY_10HZ:
    case HX710C_CLOCK_FFREQUENCY_40HZ:
        _clockFrequency = clock_frequency;
        read(); //  下一次切换
        return true;
    }
    return false; //  unchanged, but incorrect value.
}

uint8_t hx710c::get_clock_frequency()
{
    return _clockFrequency;
}

bool hx710c::set_scale(float scale)
{
    if (scale == 0)
        return false;
    _scale = 1.0 / scale;
    return true;
}

float hx710c::get_scale()
{
    return 1.0 / _scale;
}

void hx710c::set_offset(long offset)
{
    _offset = offset;
}

long hx710c::get_offset()
{
    return _offset;
}

void hx710c::calibrate_scale(uint16_t weight, uint8_t times)
{
    _scale = (1.0 * weight) / (read_average(times) - _offset);
}

void hx710c::power_down()
{
    //  至少 60 us 高电平
  digitalWrite(_clockPin, HIGH);
  delayMicroseconds(64);
}

void hx710c::power_up()
{
    digitalWrite(_clockPin, LOW);
}

uint32_t hx710c::last_read()
{
    return _lastRead;
}

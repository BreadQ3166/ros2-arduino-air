#include "tvoc_sensor.h"

#define        COV_RATIO                       0.172
#define        NO_DUST_VOLTAGE                 400
#define        SYS_VOLTAGE                     5020.0

const int iled = 7;
const int vout = A0;

// 变量定义
float density = 0;
float voltage = 0;
float avgADC = 0;
int32_t aq_grade = 1;

// 定时器变量
unsigned long lastSampleTime = 0;   // 采样计时
unsigned long lastReportTime = 0;   // 上报计时
const unsigned long sampleInterval = 100;  // 100ms 采样一次灰尘 (10Hz)
const unsigned long reportInterval = 1000; // 1000ms 发送一次数据 (1Hz)

// --- 优化后的滤波函数 (内部逻辑不变，改为非阻塞调用) ---
float updateSmoothADC() {
  const int FILTER_SIZE = 60;          
  static float _buff[FILTER_SIZE];     
  static float _sum = 0;               
  static int _index = 0;               
  static bool _full = false;           

  // 1. 采集原始值
  digitalWrite(iled, HIGH);
  delayMicroseconds(280); 
  int rawADC = analogRead(vout);
  digitalWrite(iled, LOW);
  delayMicroseconds(40); 

  // 2. 更新循环队列
  if (!_full) {
    _buff[_index] = rawADC;
    _sum += rawADC;
    _index++;
    if (_index >= FILTER_SIZE) {
      _full = true;
      _index = 0;
    }
    return (float)_sum / _index;
  } else {
    _sum -= _buff[_index];
    _buff[_index] = rawADC;
    _sum += rawADC;
    _index = (_index + 1) % FILTER_SIZE;
    return (float)_sum / FILTER_SIZE;
  }
}

// --- 1. 定义空气质量阈值矩阵 (方便随时修改) ---
// 这里的数字对应：[良(2级), 轻度(3级), 中重度(4级), 严重(5级)]
const float THRES_TVOC[]  = {0.3,  0.6,  1.5,  2.0};   // mg/m3
const int   THRES_CO2[]   = {700,  1000, 1500, 2500};  // ppm
const float THRES_DUST[]  = {35.0, 75.0, 115.0, 150.0};// ug/m3
const int   THRES_CH2O[]  = {40,   64,   150,  300};   // ppb (64ppb≈0.08mg/m3)

// 辅助函数：计算单项指标的等级
int calculateSingleGrade(float value, const float thresholds[]) {
  if (value < thresholds[0]) return 1; // 优
  if (value < thresholds[1]) return 2; // 良
  if (value < thresholds[2]) return 3; // 轻度
  if (value < thresholds[3]) return 4; // 中重度
  return 5;                            // 严重
}

// 整数版重载 (用于 CO2, CH2O 等)
int calculateSingleGrade(int value, const int thresholds[]) {
  if (value < thresholds[0]) return 1;
  if (value < thresholds[1]) return 2;
  if (value < thresholds[2]) return 3;
  if (value < thresholds[3]) return 4;
  return 5;
}

// --- 2. 在主逻辑 loop 中调用 ---
void updateAQGrade() {
  // 分别计算各传感器的当前等级
  int g_tvoc  = calculateSingleGrade(tvoc, THRES_TVOC);
  int g_co2   = calculateSingleGrade(co2, THRES_CO2);
  int g_dust  = calculateSingleGrade(density, THRES_DUST);
  int g_ch2o  = calculateSingleGrade(ch2o, THRES_CH2O);

  // --- 短板效应核心：取所有等级中的最大值 ---
  int final_g = g_tvoc;
  if (g_co2 > final_g)  final_g = g_co2;
  if (g_dust > final_g) final_g = g_dust;
  if (g_ch2o > final_g) final_g = g_ch2o;

  aq_grade = final_g; // 更新全局变量
}

void setup() {
  pinMode(iled, OUTPUT);
  digitalWrite(iled, LOW);
  
  Serial.begin(115200);
  Serial.println("Mega 2560 Pseudo-RTOS System 1Hz Start...");

  tvoc_init();
  tvoc_set_device_active_mode();
  
  randomSeed(analogRead(0));
}

void loop() {
  unsigned long currentMillis = millis();

  // --- 任务1：高频采样灰尘传感器 (非阻塞) ---
  // 每 10ms 执行一次，保证 1 秒内能完成约 100 次采样，填满 60 个样本的滤波器
  if (currentMillis - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentMillis;
    avgADC = updateSmoothADC(); 
  }

  // --- 任务2：低频上报数据 (1Hz) ---
  if (currentMillis - lastReportTime >= reportInterval) {
    lastReportTime = currentMillis;

    // 1. 计算灰尘浓度
    voltage = (avgADC * SYS_VOLTAGE / 1024.0) * 11.0;
    if (voltage >= NO_DUST_VOLTAGE) {
      density = (voltage - NO_DUST_VOLTAGE) * COV_RATIO;
    } else {
      density = 0;
    }

    // 2. 获取 TVOC/CO2/CH2O 数据
    // 注意：tvoc_get_active_device_data() 通常内部处理串口读取，1Hz 频率非常安全
    tvoc_get_active_device_data();

    // 3. 科学空气等级判断 (基于前述讨论的优化逻辑)
    
    // 使用“短板效应”判断
    updateAQGrade();

    // 4. 组装并按照协议发送数据包
    // 格式: $START,AQ,DUST,TVOC,CO2,CH2O,$END
    Serial.print("$START,");
    
    Serial.print(aq_grade);      // [1] 空气等级
    Serial.print(",");
    
    Serial.print(density, 2);    // [2] 灰尘 (ug/m3)
    Serial.print(",");
    
    Serial.print(tvoc, 2);       // [3] TVOC (ppm)
    Serial.print(",");
    
    Serial.print(co2);           // [4] CO2 (ppm)
    Serial.print(",");
    
    Serial.print(ch2o);          // [5] 甲醛 (ppb)
    
    Serial.println(",$END");     // [6] 结尾
  }

}
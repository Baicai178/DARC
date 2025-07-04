#include <Adafruit_DPS310.h>
#include <lvgl.h>
#include <Wire.h>
#include <Arduino.h>
#include "dps310.h"
#include "ui.h"

// ------------------------------
// 硬件GPIO定义
// ------------------------------
#define AIR_PUMP_GPIO 17      // 气泵控制引脚
#define SOLENOID_VALVE_GPIO 18  // 电磁阀控制引脚

// ------------------------------
// SDP810相关变量（保留你的定义）
// ------------------------------
#define SDP810_CHART_POINT_MAX 50
static lv_coord_t sdp810_pressure_buffer[SDP810_CHART_POINT_MAX] = {0};
static int sdp810_buffer_idx = 0;
static lv_chart_series_t *sdp810_chart_series = NULL;

// ------------------------------
// DPS310相关变量（带前缀，避免冲突）
// ------------------------------
#define DPS310_CHART_POINT_MAX 50
static lv_coord_t dps310_pressure_buffer[DPS310_CHART_POINT_MAX] = {0};
static int dps310_buffer_idx = 0;
static lv_chart_series_t *dps310_chart_series = NULL;

// DPS310传感器对象
Adafruit_DPS310 dps;

// ------------------------------
// 测试状态管理
// ------------------------------
enum TestState {
  TEST_IDLE,
  TEST_INFLATING,
  TEST_STABILIZING,
  TEST_TESTING,
  TEST_PASSED,
  TEST_FAILED
};

TestState current_test_state = TEST_IDLE;
unsigned long test_start_time = 0;
unsigned long test_duration = 10000; // 默认10秒测试时间
float target_pressure = 1000.0;      // 默认目标压力 1000Pa
float pass_threshold = 50.0;         // 默认通过阈值 50Pa
float baseline_pressure = 0.0;       // 基线压力
float leak_rate = 0.0;               // 泄漏率
int progress_percent = 0;            // 测试进度百分比

// 泄漏率历史记录
float leak_history[4] = {0.0, 0.0, 0.0, 0.0}; // n32, n22, n12, n02

// 显示单位枚举
enum PressureUnit {
  UNIT_PA,
  UNIT_KPA,
  UNIT_HPA,
  UNIT_MPA
};

PressureUnit current_unit = UNIT_PA;
bool blink_state = false;
unsigned long last_blink_time = 0;

// ------------------------------
// 辅助函数
// ------------------------------
float convert_pressure_unit(float pressure_pa, PressureUnit unit) {
  switch (unit) {
    case UNIT_PA:  return pressure_pa;
    case UNIT_KPA: return pressure_pa / 1000.0;
    case UNIT_HPA: return pressure_pa / 100.0;
    case UNIT_MPA: return pressure_pa / 1000000.0;
    default: return pressure_pa;
  }
}

const char* get_unit_string(PressureUnit unit) {
  switch (unit) {
    case UNIT_PA:  return "Pa";
    case UNIT_KPA: return "kPa";
    case UNIT_HPA: return "hPa";
    case UNIT_MPA: return "MPa";
    default: return "Pa";
  }
}

// ------------------------------
// 硬件控制函数
// ------------------------------
void control_air_pump(bool state) {
  digitalWrite(AIR_PUMP_GPIO, state ? HIGH : LOW);
  Serial.printf("气泵 %s\n", state ? "开启" : "关闭");
}

void control_solenoid_valve(bool state) {
  digitalWrite(SOLENOID_VALVE_GPIO, state ? HIGH : LOW);
  Serial.printf("电磁阀 %s\n", state ? "开启" : "关闭");
}

// ------------------------------
// 初始化DPS310
// ------------------------------
bool init_dps310() {
  if (!dps.begin_I2C()) {
    Serial.println("DPS310初始化失败!");
    return false;
  }
  
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  
  Serial.println("DPS310初始化成功");
  return true;
}

// ------------------------------
// 硬件初始化
// ------------------------------
void init_hardware() {
  // 初始化GPIO
  pinMode(AIR_PUMP_GPIO, OUTPUT);
  pinMode(SOLENOID_VALVE_GPIO, OUTPUT);
  
  // 初始状态关闭
  control_air_pump(false);
  control_solenoid_valve(false);
  
  // 初始化DPS310
  if (!init_dps310()) {
    Serial.println("传感器初始化失败!");
  }
}

// ------------------------------
// UI更新函数
// ------------------------------
void update_progress_bar(int percent) {
  if (ui_Bar9) {
    lv_bar_set_value(ui_Bar9, percent, LV_ANIM_ON);
  }
  
  if (ui_jc2) {
    lv_label_set_text_fmt(ui_jc2, "%d%%", percent);
  }
}

void update_test_indicator(TestState state) {
  if (!ui_NGLED2 || !ui_NGPS2) return;
  
  switch (state) {
    case TEST_IDLE:
      lv_obj_set_style_bg_color(ui_NGLED2, lv_color_white(), 0);
      lv_label_set_text(ui_NGPS2, "待机");
      break;
      
    case TEST_INFLATING:
      lv_obj_set_style_bg_color(ui_NGLED2, lv_color_hex(0x87CEEB), 0); // 浅蓝色
      lv_label_set_text(ui_NGPS2, "充气中");
      break;
      
    case TEST_STABILIZING:
    case TEST_TESTING:
      if (blink_state) {
        lv_obj_set_style_bg_color(ui_NGLED2, lv_color_hex(0x90EE90), 0); // 浅绿色
      } else {
        lv_obj_set_style_bg_color(ui_NGLED2, lv_color_white(), 0);
      }
      lv_label_set_text(ui_NGPS2, "测试中");
      break;
      
    case TEST_PASSED:
      lv_obj_set_style_bg_color(ui_NGLED2, lv_color_hex(0x00FF00), 0); // 绿色
      lv_label_set_text(ui_NGPS2, "PASS");
      break;
      
    case TEST_FAILED:
      if (blink_state) {
        lv_obj_set_style_bg_color(ui_NGLED2, lv_color_hex(0xFF0000), 0); // 红色
      } else {
        lv_obj_set_style_bg_color(ui_NGLED2, lv_color_white(), 0);
      }
      lv_label_set_text(ui_NGPS2, "NG");
      break;
  }
}

void update_leak_rate_display() {
  if (ui_n32) {
    lv_label_set_text_fmt(ui_n32, "%.2f %s", 
      convert_pressure_unit(leak_history[0], current_unit), 
      get_unit_string(current_unit));
  }
  
  if (ui_n22) {
    lv_label_set_text_fmt(ui_n22, "%.2f %s", 
      convert_pressure_unit(leak_history[1], current_unit), 
      get_unit_string(current_unit));
  }
  
  if (ui_n12) {
    lv_label_set_text_fmt(ui_n12, "%.2f %s", 
      convert_pressure_unit(leak_history[2], current_unit), 
      get_unit_string(current_unit));
  }
  
  if (ui_n02) {
    lv_label_set_text_fmt(ui_n02, "%.2f %s", 
      convert_pressure_unit(leak_history[3], current_unit), 
      get_unit_string(current_unit));
  }
}

void update_progress_bar_style(TestState state) {
  if (!ui_Bar9) return;
  
  switch (state) {
    case TEST_INFLATING:
      lv_obj_set_style_bg_color(ui_Bar9, lv_color_hex(0x87CEEB), LV_PART_INDICATOR); // 浅蓝色
      break;
      
    case TEST_TESTING:
      if (blink_state) {
        lv_obj_set_style_bg_color(ui_Bar9, lv_color_hex(0x90EE90), LV_PART_INDICATOR); // 浅绿色
      }
      break;
      
    case TEST_PASSED:
      lv_obj_set_style_bg_color(ui_Bar9, lv_color_hex(0x00FF00), LV_PART_INDICATOR); // 绿色
      break;
      
    case TEST_FAILED:
      if (blink_state) {
        lv_obj_set_style_bg_color(ui_Bar9, lv_color_hex(0xFF8C00), LV_PART_INDICATOR); // 橙色
      }
      break;
  }
}

// ------------------------------
// 测试流程控制
// ------------------------------
void start_test() {
  if (current_test_state != TEST_IDLE) return;
  
  Serial.println("开始测试");
  current_test_state = TEST_INFLATING;
  test_start_time = millis();
  progress_percent = 0;
  
  // 开始充气
  control_air_pump(true);
  control_solenoid_valve(false);
  
  // 清空图表缓冲区
  memset(dps310_pressure_buffer, 0, sizeof(dps310_pressure_buffer));
  dps310_buffer_idx = 0;
  
  update_test_indicator(current_test_state);
  update_progress_bar(0);
}

void stop_test() {
  Serial.println("停止测试");
  current_test_state = TEST_IDLE;
  
  // 关闭硬件
  control_air_pump(false);
  control_solenoid_valve(false);
  
  update_test_indicator(current_test_state);
  update_progress_bar(0);
}

void update_test_process() {
  if (current_test_state == TEST_IDLE) return;
  
  unsigned long elapsed = millis() - test_start_time;
  float current_pressure = get_current_pressure();
  
  switch (current_test_state) {
    case TEST_INFLATING:
      // 充气阶段 - 当压力接近目标时停止充气
      if (current_pressure >= target_pressure * 0.95) {
        control_air_pump(false);
        current_test_state = TEST_STABILIZING;
        test_start_time = millis(); // 重置时间
        baseline_pressure = current_pressure;
        Serial.printf("充气完成，基线压力: %.2f Pa\n", baseline_pressure);
      }
      progress_percent = min(100, (int)(current_pressure / target_pressure * 100));
      break;
      
    case TEST_STABILIZING:
      // 稳定阶段 - 等待2秒稳定
      if (elapsed >= 2000) {
        current_test_state = TEST_TESTING;
        test_start_time = millis();
        baseline_pressure = current_pressure;
        Serial.printf("开始测试，基线压力: %.2f Pa\n", baseline_pressure);
      }
      progress_percent = min(100, (int)(elapsed / 20.0)); // 2秒内完成
      break;
      
    case TEST_TESTING:
      // 测试阶段
      if (elapsed >= test_duration) {
        // 计算泄漏率
        leak_rate = baseline_pressure - current_pressure;
        
        // 更新历史记录
        for (int i = 0; i < 3; i++) {
          leak_history[i] = leak_history[i + 1];
        }
        leak_history[3] = leak_rate;
        
        // 判断测试结果
        if (leak_rate <= pass_threshold) {
          current_test_state = TEST_PASSED;
          Serial.printf("测试通过，泄漏率: %.2f Pa\n", leak_rate);
        } else {
          current_test_state = TEST_FAILED;
          Serial.printf("测试失败，泄漏率: %.2f Pa\n", leak_rate);
        }
        
        update_leak_rate_display();
      }
      progress_percent = min(100, (int)(elapsed * 100 / test_duration));
      break;
      
    case TEST_PASSED:
    case TEST_FAILED:
      // 测试结束状态
      progress_percent = 100;
      break;
  }
  
  update_progress_bar(progress_percent);
  update_test_indicator(current_test_state);
  update_progress_bar_style(current_test_state);
}

// ------------------------------
// 获取当前压力值
// ------------------------------
float get_current_pressure() {
  sensors_event_t temp_event, pressure_event;
  dps.getEvents(&temp_event, &pressure_event);
  
  float pressure_hpa = pressure_event.pressure;
  if (isnan(pressure_hpa)) {
    return 0.0;
  }
  
  return pressure_hpa * 100.0f; // 转换为Pa
}

// ------------------------------
// 更新图表（DPS310）
// ------------------------------
void update_chart2() {
  float current_pressure = get_current_pressure();
  
  if (current_pressure == 0.0) {
    Serial.println("⚠️ 读取气压失败");
    return;
  }
  
  // 转换为显示单位
  float display_pressure = convert_pressure_unit(current_pressure, current_unit);
  
  Serial.printf("DPS310 实时气压: %.2f %s\n", display_pressure, get_unit_string(current_unit));
  
  // 存入缓冲区
  dps310_pressure_buffer[dps310_buffer_idx] = (lv_coord_t)current_pressure;
  dps310_buffer_idx = (dps310_buffer_idx + 1) % DPS310_CHART_POINT_MAX;
  
  // 获取Chart对象
  lv_obj_t *chart = ui_Chart2;
  if (!chart) return;
  
  // 初始化图表系列
  if (!dps310_chart_series) {
    dps310_chart_series = lv_chart_add_series(chart, lv_color_hex(0x0066FF), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_set_point_count(chart, DPS310_CHART_POINT_MAX);
  }
  
  // 动态设置显示范围
  int32_t range_min, range_max;
  
  if (current_test_state == TEST_IDLE) {
    // 待机状态：标准大气压附近
    range_min = 101000;  // 约1010hPa
    range_max = 101700;  // 约1017hPa
  } else if (current_test_state == TEST_INFLATING) {
    // 充气阶段：从大气压到目标压力
    range_min = 101000;
    range_max = (int32_t)(target_pressure + 101325 + 200);
  } else {
    // 测试阶段：以目标压力为中心
    float center = baseline_pressure + 101325;
    range_min = (int32_t)(center - 500);
    range_max = (int32_t)(center + 500);
  }
  
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, range_min, range_max);
  
  // 更新图表
  lv_chart_set_ext_y_array(chart, dps310_chart_series, dps310_pressure_buffer);
  lv_chart_refresh(chart);
}

// ------------------------------
// 按钮事件处理
// ------------------------------
void button_start_event_handler(lv_event_t * e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    start_test();
  }
}

void button_stop_event_handler(lv_event_t * e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    stop_test();
  }
}

// ------------------------------
// 下拉菜单事件处理
// ------------------------------
void dropdown_time_event_handler(lv_event_t * e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    lv_obj_t * dropdown = lv_event_get_target(e);
    uint16_t selected = lv_dropdown_get_selected(dropdown);
    
    switch (selected) {
      case 0: test_duration = 5000; break;   // 5秒
      case 1: test_duration = 10000; break;  // 10秒
      case 2: test_duration = 15000; break;  // 15秒
      case 3: test_duration = 30000; break;  // 30秒
      case 4: test_duration = 60000; break;  // 60秒
    }
    
    Serial.printf("测试时间设置为: %lu 毫秒\n", test_duration);
  }
}

void dropdown_target_event_handler(lv_event_t * e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    lv_obj_t * dropdown = lv_event_get_target(e);
    uint16_t selected = lv_dropdown_get_selected(dropdown);
    
    switch (selected) {
      case 0: target_pressure = 500.0; break;   // 500Pa
      case 1: target_pressure = 1000.0; break;  // 1000Pa
      case 2: target_pressure = 1500.0; break;  // 1500Pa
      case 3: target_pressure = 2000.0; break;  // 2000Pa
      case 4: target_pressure = 3000.0; break;  // 3000Pa
    }
    
    Serial.printf("目标压力设置为: %.1f Pa\n", target_pressure);
  }
}

void dropdown_threshold_event_handler(lv_event_t * e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    lv_obj_t * dropdown = lv_event_get_target(e);
    uint16_t selected = lv_dropdown_get_selected(dropdown);
    
    switch (selected) {
      case 0: pass_threshold = 10.0; break;   // 10Pa
      case 1: pass_threshold = 20.0; break;   // 20Pa
      case 2: pass_threshold = 50.0; break;   // 50Pa
      case 3: pass_threshold = 100.0; break;  // 100Pa
      case 4: pass_threshold = 200.0; break;  // 200Pa
    }
    
    Serial.printf("通过阈值设置为: %.1f Pa\n", pass_threshold);
  }
}

void dropdown_unit_event_handler(lv_event_t * e) {
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    lv_obj_t * dropdown = lv_event_get_target(e);
    uint16_t selected = lv_dropdown_get_selected(dropdown);
    
    current_unit = (PressureUnit)selected;
    Serial.printf("显示单位设置为: %s\n", get_unit_string(current_unit));
    
    // 更新显示
    update_leak_rate_display();
  }
}

// ------------------------------
// 初始化UI事件
// ------------------------------
void init_ui_events() {
  if (ui_Button21) {
    lv_obj_add_event_cb(ui_Button21, button_start_event_handler, LV_EVENT_CLICKED, NULL);
  }
  
  if (ui_Button16) {
    lv_obj_add_event_cb(ui_Button16, button_stop_event_handler, LV_EVENT_CLICKED, NULL);
  }
  
  if (ui_time2) {
    lv_obj_add_event_cb(ui_time2, dropdown_time_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
  }
  
  if (ui_tpp2) {
    lv_obj_add_event_cb(ui_tpp2, dropdown_target_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
  }
  
  if (ui_tss2) {
    lv_obj_add_event_cb(ui_tss2, dropdown_threshold_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
  }
  
  if (ui_unit2) {
    lv_obj_add_event_cb(ui_unit2, dropdown_unit_event_handler, LV_EVENT_VALUE_CHANGED, NULL);
  }
}

// ------------------------------
// 主程序
// ------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("DPS310 气压测试系统启动");
  
  // 初始化硬件
  init_hardware();
  
  // 初始化UI（假设已经在其他地方初始化）
  // ui_init();
  
  // 初始化UI事件
  init_ui_events();
  
  // 初始化显示
  update_test_indicator(TEST_IDLE);
  update_progress_bar(0);
  update_leak_rate_display();
  
  Serial.println("系统初始化完成");
}

void loop() {
  // 更新测试流程
  update_test_process();
  
  // 更新图表
  update_chart2();
  
  // 处理闪烁逻辑
  if (millis() - last_blink_time > 500) {
    blink_state = !blink_state;
    last_blink_time = millis();
    
    // 更新需要闪烁的状态
    if (current_test_state == TEST_TESTING || current_test_state == TEST_FAILED) {
      update_test_indicator(current_test_state);
      update_progress_bar_style(current_test_state);
    }
  }
  
  // LVGL任务处理
  lv_task_handler();
  
  delay(50); // 20Hz更新频率
}
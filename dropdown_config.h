#ifndef DROPDOWN_CONFIG_H
#define DROPDOWN_CONFIG_H

// 测试时间选项 (time2)
// 对应的值：5秒, 10秒, 15秒, 30秒, 60秒
#define TIME_OPTIONS "5s\n10s\n15s\n30s\n60s"

// 充气目标选项 (tpp2) 
// 对应的值：500Pa, 1000Pa, 1500Pa, 2000Pa, 3000Pa
#define TARGET_PRESSURE_OPTIONS "500Pa\n1000Pa\n1500Pa\n2000Pa\n3000Pa"

// 通过条件选项 (tss2)
// 对应的值：10Pa, 20Pa, 50Pa, 100Pa, 200Pa
#define PASS_THRESHOLD_OPTIONS "10Pa\n20Pa\n50Pa\n100Pa\n200Pa"

// 显示单位选项 (unit2)
// 对应的值：Pa, kPa, hPa, MPa
#define UNIT_OPTIONS "Pa\nkPa\nhPa\nMPa"

// 默认选项索引
#define DEFAULT_TIME_INDEX 1        // 10秒
#define DEFAULT_TARGET_INDEX 1      // 1000Pa
#define DEFAULT_THRESHOLD_INDEX 2   // 50Pa
#define DEFAULT_UNIT_INDEX 0        // Pa

// 压力测试范围配置
#define MIN_TARGET_PRESSURE 100.0   // 最小目标压力 (Pa)
#define MAX_TARGET_PRESSURE 5000.0  // 最大目标压力 (Pa)
#define MIN_TEST_DURATION 3000      // 最小测试时间 (ms)
#define MAX_TEST_DURATION 120000    // 最大测试时间 (ms)

// DPS310传感器配置
#define DPS310_PRESSURE_RANGE_MIN 300.0   // hPa (约300hPa = 30000Pa)
#define DPS310_PRESSURE_RANGE_MAX 1200.0  // hPa (约1200hPa = 120000Pa)
#define DPS310_ACCURACY 0.06              // hPa (典型精度)

// 图表显示配置
#define CHART_IDLE_RANGE 700         // 待机状态图表范围 (Pa)
#define CHART_TEST_RANGE 1000        // 测试状态图表范围 (Pa)
#define CHART_INFLATION_MARGIN 200   // 充气阶段图表边距 (Pa)

// 测试流程时间配置
#define STABILIZATION_TIME 2000      // 稳定时间 (ms)
#define INFLATION_TIMEOUT 30000      // 充气超时时间 (ms)
#define PRESSURE_TOLERANCE 0.95      // 压力到达容差 (95%目标压力)

// 闪烁配置
#define BLINK_INTERVAL 500           // 闪烁间隔 (ms)

// 颜色配置
#define COLOR_IDLE 0xFFFFFF          // 白色
#define COLOR_INFLATING 0x87CEEB     // 浅蓝色
#define COLOR_TESTING 0x90EE90       // 浅绿色
#define COLOR_PASSED 0x00FF00        // 绿色
#define COLOR_FAILED 0xFF0000        // 红色
#define COLOR_FAILED_BLINK 0xFF8C00  // 橙色

#endif // DROPDOWN_CONFIG_H
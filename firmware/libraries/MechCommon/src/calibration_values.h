#pragma once
#include <stdint.h>
// Inherited scaling is provisional: measure this rig before quantitative use.
constexpr int DAC_CENTRE = 1450;
constexpr int DAC_HALF_RANGE = 1526;
constexpr float DAC_CORR_OFFSET = 49.0f;
constexpr float DAC_CORR_GAIN = 4002.0f;
constexpr float POT_RAW_AT_CENTRE = 1414.0f;
constexpr float POT_DAC_COUNTS_PER_ADC = 5.0118f;
constexpr int CALIBRATION_TARGET_JRK_SCALE = 1900;
constexpr int DAC_REG_MIN = 0;
constexpr int DAC_REG_MAX = 4095;
// Applied DAC commands, NOT inferred mechanical limits. -1 disables movement.
// Supply measured/configured rig values here; no automatic sweep or persistence.
constexpr int CAL_MIN_DAC = -1;
constexpr int CAL_MAX_DAC = -1;
static_assert(CAL_MIN_DAC == -1 || (CAL_MIN_DAC >= 0 && CAL_MIN_DAC <= 4095), "invalid CAL_MIN_DAC");
static_assert(CAL_MAX_DAC == -1 || (CAL_MAX_DAC >= 0 && CAL_MAX_DAC <= 4095), "invalid CAL_MAX_DAC");
constexpr float ACTUATOR_STROKE_MM = 300.0f;
constexpr float CYLINDER_DIAMETER_MM = 115.0f;
constexpr float V_TIDAL_ML = 700.0f;
constexpr float IE_EXP_PARTS = 2.0f;
constexpr float IE_INSP_PARTS = 1.0f;
constexpr uint32_t LOG_INTERVAL_US = 10000;
constexpr float SFM3000_SCALE_FLOW_AIR = 140.0f;

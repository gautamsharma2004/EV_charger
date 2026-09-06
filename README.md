# ⚡ Programmable Battery Charger Firmware - PY32F002A

**Production-grade battery charger firmware** with state machine control, thermal protection, dynamic SOC estimation, and soft-start charging. Supports 48–72V battery packs (Li-Ion, LiFePO₄, and compatible chemistries) within configurable voltage/current ranges.

**Platform:** PY32F002AF15P6 (32-bit ARM Cortex-M0+) | **Power Stage:** PWM control via TIM1 CH4 (TL494 feedback external)  
**Voltage Range:** 54–72V (user-configurable) | **Current Range:** 6.2A-10.2A (software-limited)  
**Feature Set:** CC/CV charging, dynamic SOC tracking, real-time thermal derating, safety timeouts, multi-chemistry support

---

## 📋 Project Overview

This is a complete **embedded systems firmware project** for a programmable multi-chemistry battery charger, developed during an intensive internship focused on power electronics and embedded systems design. The project demonstrates:

- **Real-time state machine**: 11-state FSM handling battery detection, CC/CV charging, thermal protection, and fault recovery
- **Comprehensive sensing & protection**: Multi-sensor fusion (voltage, current, thermal), safety timeouts (charge session limit), and thermal derating
- **Hardware integration**: ADC multiplexing, NTC thermistor linearization, TM1637 display, relay control, and PWM gate control
- **Non-volatile configuration**: Persistent user settings (VSET, CSET) with auto-save UI
- **Thermal management**: Three-state thermal control (normal → derating → shutdown)
- **Multi-chemistry support**: Configurable charge profiles for Li-Ion, LiFePO₄, and hybrid batteries

**Technical Scope:** This project integrates firmware (C + ARM HAL), analog electronics (signal conditioning), power stage management (PWM duty cycle), and safety-critical systems design—applicable to battery management applications.

---

## 🎯 Key Features & Capabilities

### ⚡ Charging Control

| Feature | Specification | Implementation |
|---------|---------------|-----------------|
| **Voltage Range** | 48–72V (configurable via VSET) | User selectable, persistent storage |
| **CC Phase** | 0 → 6.2A with safety ramp | PWM feedback loop on current sense (ADC CH0) |
| **CV Phase** | Configurable setpoint regulation | PWM feedback loop on voltage sense (ADC CH5) |
| **Soft Start** | Progressive current ramp | Configurable ramp rate via `ACTIVE_TARGET_CURRENT` |
| **Dynamic VSET/CSET** | User-configurable via button UI | Persistent storage in non-volatile memory; 21-level lookup table |
| **Multi-Chemistry** | Li-Ion, LiFePO₄, NMC, NCA compatible | SOC LUT can be adapted per chemistry |
| **CV Termination** | Auto-stop when current drops to 40% of CSET for 10 seconds | Automatic transition to CHARGE_COMPLETE |

### 🌡️ Thermal Management

**Three-State Thermal Control:**

| State | Temp Range | Action | Recovery |
|-------|-----------|--------|----------|
| **NORMAL** | < 80°C | Full current output | — |
| **DERATING** | 80–90°C | Proportional current reduction | Automatic when T < 80°C |
| **SHUTDOWN** | ≥ 90°C | Charging stop; output disabled | Manual restart after cooling to ≤ 60°C |

**Sensors:**
- MOSFET NTC (PA6, ADC CH6): Gate driver MOSFET temperature
- Transformer NTC (PA4, ADC CH4): Transformer winding temperature
- **Control uses maximum of both** for conservative protection

### 🔋 State-of-Charge (SOC) Estimation

**Adaptive SOC Lookup Table (21 levels: 0–100% @ 5% intervals)**

- **Algorithm:** Voltage-based LUT with hysteresis filtering
- **Evaluation Period:** 60-second sampling window
- **Transition Rule:** Voltage must remain continuously above next SOC threshold for full 60s
- **Max Increment:** 5% per evaluation cycle
- **No Decrement Lock:** SOC never decreases while charging
- **Recalculation on Reset:** Battery remove/reconnect triggers immediate LUT lookup
- **Chemistry-Specific:** LUT can be recalibrated for different battery chemistries

**Example (Default VSET = 67.2V):**
```
SOC 0%   → 0.650 × 67.2V = 43.68V (critical low)
SOC 50%  → 0.920 × 67.2V = 61.824V (mid-charge)
SOC 100% → 1.000 × 67.2V = 67.2V (full charge)
```

### 🛡️ Protection & Safety Functions

#### 1. **Battery Detection (VSET Wake Mode)**
- **Trigger:** AC power ON or voltage jump > 2V detected
- **Duration:** Waits for voltage to rise above configured threshold
- **Output:** Regulated at configured VSET, current carefully controlled with safe limits
- **Purpose:** Deep-discharge wake-up, battery presence detection
- **Exit:** Auto-transition to charging when voltage reaches threshold

#### 2. **CV Safety Timeout**
- **Trigger:** Battery voltage reaches CV setpoint
- **Timeout:** 60 minutes maximum
- **Fail Action:** Forces abort if timeout exceeded
- **Recovery:** AC power cycle

#### 3. **Charge Session Safety**
- **Trigger:** Charge session active
- **Timeout:** 7 hours maximum total charge time
- **Fail Action:** Forces termination, enters fault state
- **Recovery:** AC power cycle

#### 4. **Thermal Protection**
- **SHUTDOWN:** Charging stops at ≥90°C
- **DERATING:** Current scales linearly 80–90°C
- **Auto-Recovery:** When temp < 80°C during derating, full current resumes

#### 5. **Battery Not Good (BTNG)**
- **Condition:** Voltage remains below 20% threshold after battery detect phase
- **Action:** No charging initiated; standby mode
- **Recovery:** Reconnect battery or AC restart

#### 6. **Short Circuit Detection (SCPT)**
- **Trigger:** Output current spike with rapid PI response saturation
- **Action:** Immediate output cutoff
- **Recovery:** Manual AC restart

---

## 🏗️ Hardware Architecture

### **Block Diagram**

```
┌─────────────────┐
│   AC Mains      │ ──[INPUT]──┐
│  110V / 220V    │            │
└─────────────────┘            │
                               │
                    ┌──────────┴──────────┐
                    │                     │
              [Bulk Capacitor]   [DC Regulation]
                    │                     │
                    └────────┬────────────┘
                             │
                    [PWM TIM1 CH4]
                             │
                  ┌──────────┴──────────┐
                  │                     │
            [External Gate Drive]  [Output Filtering]
                  │                     │
                  └────────┬────────────┘
                           │
                      [Output DC]
                        48-72V
                           │
                  ┌─────────┴──────────┐
                  │                    │
            [Current Sense]    [Voltage Sense]
            [ADC CH0]          [ADC CH5]
                  │                    │
        ┌─────────┴────────────┬───────┴─────────────┐
        │                      │                     │
    [PY32F002A MCU]       [NTC Thermistors]    [Output Relay]
    (Control FSM)         (Thermal Protection) (Battery Isolation)
        │
    ┌───┼────┬──────────┐
    │   │    │          │
  [TIM1] [ADC] [GPIO]   [TM1637 LCD]
  [PWM] [Timers] [Interrupts] [Button Input]
    │
 [LED/Fan Outputs]
```

### **Key ICs & Components**

| Component | Part Number | Function | Quantity |
|-----------|-----------|----------|----------|
| **MCU** | PY32F002AF15P6 | Main controller (24MHz OSC, ARM Cortex-M0+) | 1 |
| **PWM Output** | TIM1 CH4 | PWM duty cycle control (20kHz, 1199 steps) | 1 |
| **NTC Thermistors** | MF11-103 (10k @ 25°C) | MOSFET & transformer temperature sensing | 2 |
| **Current Sense** | WSL2512R0400FEA (shunt) | High-side current sensing (0.04Ω) | 1 |
| **Display** | TM1637 | 4-digit 7-segment LCD (VSET, CSET, Voltage, SOC) | 1 |
| **Relay** | FSM2JLH | Battery output isolation (optional disconnect) | 1 |

### **ADC Channels & Sensing**

| Channel | GPIO | Sensor | Calibration | Range |
|---------|------|--------|-------------|-------|
| CH0 | PA0 | Current Shunt (0.04Ω) | CURRENT_CAL_FACTOR (default 1.0) | 0–6.2A |
| CH4 | PA4 | Transformer NTC | Steinhart-Hart | –10…+120°C |
| CH5 | PA5 | Battery Voltage Divider | VOLTAGE_CAL_FACTOR (default 0.882) | 0–80V |
| CH6 | PA6 | MOSFET NTC | Steinhart-Hart | –10…+120°C |

**NTC Linearization (Steinhart-Hart):**
```c
float T_kelvin = 1.0 / (a + b*ln(R) + c*(ln(R))^3);
T_celsius = T_kelvin - 273.15;

// Constants tuned for MF11-103 (Beta = 3950)
```

---

## 📐 Firmware Architecture

### **State Machine (11 States)**

```c
typedef enum {
    STATE_INIT = 0,           // Power-on, FAN test, display VSET/CSET
    STATE_WAIT_BATTERY,       // Battery detection phase
    STATE_CHARGING,           // CC/CV charging loop
    STATE_CHARGE_COMPLETE,    // 100% SOC reached or CV termination
    STATE_THERMAL_FAULT,      // Shutdown at ≥90°C
    STATE_IDLE,               // Standby (no battery / post-charge)
    STATE_FAULT_LOCK,         // Generic fault (latched)
    STATE_MAINS_FAULT,        // AC input issue (reserved)
    STATE_BTNG,               // Battery Not Good
    STATE_CHTO,               // Charge Timeout
    STATE_SCPT                // Short Circuit Protection Triggered
} ChargerState_t;
```

**State Transitions (Implemented):**

| From | Trigger | To | Action |
|------|---------|-----|--------|
| INIT | AC ON / Power-up | WAIT_BATTERY | Start battery detection; FAN ON |
| WAIT_BATTERY | Voltage > threshold | CHARGING | Begin CC/CV control |
| WAIT_BATTERY | Voltage < 20% threshold | BTNG | Battery not detected; standby |
| CHARGING | CV + current < termination threshold for 10s | CHARGE_COMPLETE | Stop charging; hold state |
| CHARGING | Temp ≥ 90°C | THERMAL_FAULT | Output off; wait for cool-down |
| IDLE | Voltage jump > 2V detected | WAIT_BATTERY | Restart detection phase |

### **Control Loops (PI Feedback)**

#### **Voltage Control (CV Phase)**
```c
PI_Controller cv_pi = {
    .Kp = 20.0f,           // Proportional gain
    .Ki = 0.5f,            // Integral gain
    .integral_sum = 1199.0f, // Initialize to max (PWM zero output)
    .out_max = 1199.0f,    // TIM1 duty cycle limit
    .out_min = 0.0f
};

// Asymmetrical error clamp: Prevent violent power surges
error = actual - setpoint;
if (error < -2.0f) error = -2.0f;  // Only clamp downward

output = Calculate_PI(&cv_pi, ACTIVE_TARGET_VOLTAGE, g_actual_v);
TIM1->CCR4 = (uint16_t)output;  // Push to TIM1 duty cycle
```

**Behavior:**
- Tight regulation at configured VSET
- Asymmetrical clamp prevents inrush
- Integral term ensures zero steady-state error

#### **Current Control (CC Phase)**
```c
PI_Controller cc_pi = {
    .Kp = 10.0f,
    .Ki = 1.0f,
    .integral_sum = 1199.0f,
    .out_max = 1199.0f,
    .out_min = 0.0f
};

error = setpoint - actual;  // Inverted for current control
output = Calculate_PI(&cc_pi, ACTIVE_TARGET_CURRENT, g_actual_i);
TIM1->CCR4 = (uint16_t)output;
```

**Soft-Start Ramp:**
```c
ACTIVE_TARGET_CURRENT += 0.03f per loop (100ms)  // Smooth ramp from 0A to CSET
// CC loop tracks this ramping setpoint
```

#### **Thermal Derating (Proportional)**
```
T_max = max(T_mosfet, T_transformer)

if (T_max >= 80°C && T_max < 90°C) {
    currentScale = (90.0f - T_max) / 10.0f;  // Linear: 90°C → 0%, 80°C → 100%
    ACTIVE_TARGET_CURRENT = TARGET_CURRENT × currentScale;
}
```

### **Display & UI State Machine**

**UI Modes (Button Hold Control):**

```c
typedef enum {
    UI_STATE_NORMAL,           // Alternate voltage ↔ SOC/status every 10s
    UI_STATE_VSET,             // Edit target voltage
    UI_STATE_CSET,             // Edit target current
    UI_STATE_SAVE_DISPLAY_V,   // Confirm VSET display
    UI_STATE_SAVE_DISPLAY_C    // Confirm CSET display
} UI_State_t;
```

**Display Examples:**
```
During Charging:
  [67.2] (voltage × 10)  ←→  [**42] (SOC display) — Alternates every 10s

During Fault:
  [BTNG] / [SCPT] / [CHTO] — Solid display
  
Charge Complete:
  [100P] — 100% indicator
```

### **Timing & Sampling**

| Task | Period | Function |
|------|--------|----------|
| SysTick | 1 ms | System clock |
| ADC Conversion | Continuous | 239-cycle sampling per channel |
| FSM Update | 100 ms | State machine & PI control loop |
| Display Refresh | 100 ms | TM1637 output update |
| SOC Evaluation | 60 s | Check if voltage sustained above next threshold |
| Alternating Display | 10 s | Toggle between voltage & SOC/status |
| Thermal Check | 100 ms | Temperature monitoring & derating calc |

---

## 🔧 Core Algorithms & Implementations

### **SOC Estimation with Hysteresis**

```c
void Evaluate_SOC(void) {
    static uint32_t eval_timer = 0;
    static float min_voltage_in_window = 999.0f;
    
    if (eval_timer == 0) {
        min_voltage_in_window = g_actual_v;
    }
    
    // Track minimum voltage during 60s window
    if (g_actual_v < min_voltage_in_window) {
        min_voltage_in_window = g_actual_v;
    }
    
    eval_timer += 100;  // 100ms per FSM loop
    
    if (eval_timer >= 60000) {  // 60 seconds elapsed
        float next_soc_threshold = SOC_VSET_PCT[currentSOC + 1] * TARGET_VOLTAGE;
        
        // Only advance if voltage stayed above threshold entire window
        if (min_voltage_in_window >= next_soc_threshold && currentSOC < 100) {
            currentSOC += 5;  // Max 5% per cycle
            if (currentSOC > 100) currentSOC = 100;
        }
        eval_timer = 0;
    }
}
```

**Key Insight:** Hysteresis (voltage must sustain for full 60s) prevents SOC oscillation due to ripple or transients.

### **Soft-Start Current Ramping**

```c
void Soft_Start_Manager(void) {
    if (gState == STATE_CHARGING) {
        if (ACTIVE_TARGET_CURRENT < safe_current_limit) {
            ACTIVE_TARGET_CURRENT += 0.03f;  // ~0.3A/sec ramp
            if (ACTIVE_TARGET_CURRENT > safe_current_limit) {
                ACTIVE_TARGET_CURRENT = safe_current_limit;
            }
        }
    }
}
```

**Effect:**
- Prevents inrush to PWM duty cycle
- Smooth current rise observed by battery
- Reduces stress on power stage components

---

## 📊 Performance Specifications

### **Electrical Performance**

| Parameter | Target | Notes |
|-----------|--------|-------|
| **Output Voltage Accuracy** | ±0.5% @ configured VSET | PI loop dependent |
| **Output Current Accuracy** | ±2% @ configured CSET | Soft-start affects early phase |
| **Soft-Start Duration** | ~0.3A/sec ramp | ~20 seconds to rated 6.2A |
| **CV Termination** | 40% of CSET for 10 seconds | Automatic charge complete |
| **Thermal Response** | <1 second detection | NTC + ADC sampling |

### **Safety Response Times**

| Trigger | Detection Latency | Action Latency |
|---------|-------------------|-----------------|
| **Thermal Shutdown** | ~100ms (ADC + FSM) | <100ms output off |
| **CV Timeout (60min)** | Event-driven timer | Immediate if exceeded |
| **Charge Timeout (7hr)** | Event-driven timer | Immediate if exceeded |
| **SCPT (Unloaded)** | <100ms (PI maxed) | <10ms PWM cut |

---

## 🚀 Getting Started

### **Build & Compile**

**Prerequisites:**
- STM32CubeMX or PY32 equivalent toolchain
- Arm GNU Toolchain (arm-none-eabi-gcc)
- OpenOCD / ST-LINK V2 debugger

**Compilation:**
```bash
# Using Makefile
make clean
make all

# Output: main.elf, main.hex
```

### **Flashing**

```bash
# Via OpenOCD + ST-LINK V2
openocd -f interface/stlink-v2.cfg -f target/stm32f0x.cfg \
  -c "program main.hex verify reset exit"
```

### **Configuration Parameters** (in `main.c`)

**Battery Setpoints:**
```c
#define TARGET_VOLTAGE 67.20f  // Output voltage (volts)
#define TARGET_CURRENT 6.2f    // Max charging current (amps)
```

**Supported Voltage Configurations:**
```c
// 48V LiFePO₄ Pack (16S):
#define TARGET_VOLTAGE 48.0f

// 60V Li-Ion Pack (20S):
#define TARGET_VOLTAGE 60.0f

// 72V Li-Ion Pack (24S):
#define TARGET_VOLTAGE 72.0f
```

**PI Gains:**
```c
cv_pi.Kp = 20.0f;   // Voltage loop proportional
cv_pi.Ki = 0.5f;    // Voltage loop integral
cc_pi.Kp = 10.0f;   // Current loop proportional
cc_pi.Ki = 1.0f;    // Current loop integral
```

**Calibration Factors:**
```c
#define VOLTAGE_CAL_FACTOR 0.882f  // Adjust if measured V is off
#define CURRENT_CAL_FACTOR 1.000f  // Adjust if measured I is off
```

**Thermal Thresholds:**
```c
#define THERM_DERATE_START_TEMP 80.0f   // Derating begins
#define THERM_SHUTDOWN_TEMP 90.0f       // Fault shutdown
#define THERM_RECOVERY_TEMP 60.0f       // Cool enough to restart
```

---

## 🔍 Debug & Diagnostics

### **Serial Monitor (UART)**

Connect to PY32 USART at 115200 baud to monitor:

```
STATE_INIT: System initialization...
STATE_WAIT_BATTERY: Battery detection window
  V_out = 0.00V | I_out = 0.00A | T = 35.2°C

STATE_CHARGING (CC phase):
  V_out = 45.32V | I_out = 6.20A | SOC = 15% | T = 65.4°C

STATE_CHARGING (CV phase):
  V_out = 67.18V | I_out = 0.84A | SOC = 98% | T = 72.1°C

STATE_CHARGE_COMPLETE:
  SOC = 100% | Output locked | Awaiting battery disconnect
```

### **LED Indicators**

| LED | State | Meaning |
|-----|-------|---------|
| RED | Solid ON | Standby |
| RED | Blinking | Active charging |
| GREEN | Solid ON | Charge complete |
| RED+GREEN | Blinking | Fault condition |

---

## 🧪 Testing & Validation

### **Functional Tests (Pre-Deployment)**

- [ ] **Battery Detection:** Verify voltage thresholds trigger correctly
- [ ] **Soft-Start:** Confirm current ramps smoothly over ~20 seconds
- [ ] **CC-to-CV Transition:** Voltage reaches setpoint, current drops autonomously
- [ ] **SOC Estimation:** Verify LUT matches actual battery discharge curve
- [ ] **Thermal Derating:** Heat NTC to 80°C, confirm current scales down
- [ ] **Thermal Shutdown:** Heat to 90°C, confirm output disabled
- [ ] **CV Termination:** Current drops to termination threshold, charge stops after 10s
- [ ] **Display & UI:** VSET/CSET edit, display alternation
- [ ] **Multi-Chemistry:** Test with Li-Ion, LiFePO₄ (if LUT adjusted)

### **Stress Tests**

1. **Thermal Cycling:** Charge repeatedly, reach 85°C, recover
2. **Extended Charge:** Run full 7-hour session to verify timeout
3. **Load Transients:** Battery under varying load
4. **Chemistry Switching:** Switch SOC LUT, verify accuracy

---

## 🎓 Technical Depth & Learning Outcomes

### **Power Electronics**
- PWM duty cycle control for voltage/current regulation
- PI feedback loop tuning for tight regulation
- Soft-start current ramping to limit inrush stress
- Thermal management via proportional derating

### **Embedded Systems**
- State machine design (11-state FSM)
- Real-time task scheduling (1ms tick, 100ms FSM)
- ADC multiplexing with software linearization (Steinhart-Hart)
- Non-volatile storage management (EEPROM)

### **Systems Integration**
- Multi-sensor fusion (voltage, current, temperature)
- Safety-critical timeout handling
- Fault recovery strategies
- User interface state machine
- Multi-chemistry adaptability

---

## 📁 File Structure

```
battery-charger-firmware/
├── main.c                      (2102 lines)
│   ├── ADC Driver
│   ├── NTC Thermistor Module
│   ├── Battery Detection Logic
│   ├── Fan Controller
│   ├── Thermal Protection
│   ├── PI Control Loops
│   ├── Main State Machine (11-state)
│   ├── Display / UI Logic
│   └── Interrupt Handlers
│
└── main.h
    ├── Configuration Macros
    ├── Function Prototypes
    └── Module Headers
```

---

## ⚠️ **IMPORTANT NOTES ON THIS FIRMWARE VERSION**

### **AC Input Protection Status: NOT IMPLEMENTED**

**AC Over-Voltage Protection (OVPT) and AC Under-Voltage Protection (UNVP) are NOT functional in this version.**

The hardware includes PC817 optoisolators for isolation, but the firmware does **not** currently:
- Monitor AC mains voltage deviations
- Implement OVPT detection or response
- Implement UNVP detection or response
- Trigger STATE_MAINS_FAULT on AC events

**Current Code Status:**
- Line 2081-2102 in `main.c` contains commented-out AC mains protection logic
- PC817 inputs (if wired) are not read or processed
- AC protection would require additional GPIO pins and logic to implement

**To Enable AC Protection:**
1. Wire AC fault signals to GPIO pins (e.g., PA3)
2. Configure GPIO as input with pull-up/debounce
3. Uncomment and complete mains protection logic in SysTick_Handler
4. Add state transitions for OVPT/UNVP events
5. Test with mains simulation equipment

---

## 🔐 Known Limitations

### **Fully Implemented:**
✓ CC/CV charging with soft-start  
✓ Dual PI feedback (voltage & current control)  
✓ Thermal derating (80–90°C proportional)  
✓ SOC estimation (21-level LUT with 60s hysteresis)  
✓ Battery detection (voltage-based wake)  
✓ Safety timeouts (7-hour charge, 60-min CV)  
✓ 11-state fault-tolerant state machine  
✓ TM1637 display + button UI  
✓ Persistent VSET/CSET storage (EEPROM)  
✓ Multi-voltage support (48–72V configurable)  

### **NOT Implemented (Future Work):**
- [ ] AC Over-Voltage Protection (OVPT) — **Hardware ready, firmware stub only**
- [ ] AC Under-Voltage Protection (UNVP) — **Hardware ready, firmware stub only**
- [ ] CAN/Modbus remote monitoring
- [ ] BMS communication
- [ ] Pre-charge sequencing
- [ ] Multiple battery bank switching
- [ ] Graphical display support
- [ ] Wireless telemetry

---

## 📚 References & Standards

- **Thermal Control:** MIL-STD-810G (environmental testing)
- **NTC Linearization:** Steinhart-Hart equation
- **PWM Control:** PY32F002A TIM1 module (20kHz switching)
- **Chemistry References:**
  - Li-Ion (LCO, NMC, NCA): 4.2V/cell nominal
  - LiFePO₄: 3.6V/cell nominal

---

## 📄 License

**MIT License (2025)**

Permission is granted to use, modify, and distribute this firmware for educational, research, and commercial purposes.

---

## ✍️ Author & Acknowledgments

**Firmware Developer:** Gautam Sharma  
**Project Type:** Embedded Systems Internship  
**Focus Areas:** Power Electronics, Firmware Control, Safety-Critical Systems

---

**Project Status:** ✅ **Functional** (v1.0, Sep 2025)

*Last Updated:* September 2025 | *Platform:* PY32F002A | *Voltage Range:* 48–72V | *Multi-Chemistry:* Configurable

---

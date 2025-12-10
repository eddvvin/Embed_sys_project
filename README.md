Capstone Project for ECE-4380-001
# Smart Home Energy Monitoring System with Custom RTOS
**STM32-Based Real-Time Energy Monitor with IoT Integration**

## 🎯 Overview

This project implements a **real-time energy monitoring system** for smart homes, featuring:

- **Custom RTOS** built from scratch (no FreeRTOS or external libraries)
- **Real-time current measurement** using ACS712 sensor
- **Priority-based cooperative scheduler** managing 4 concurrent tasks
- **IoT integration** via ESP32 → Firebase → Web Dashboard
- **Remote relay control** through web interface
- **Historical data visualization** with date filtering

**Academic Context:** This is a capstone project demonstrating embedded systems design, real-time operating systems, hardware-software integration, and IoT connectivity.

---

## ✨ Features

### Core Features

- ✅ **Custom RTOS Implementation**
  - Priority-based cooperative scheduler
  - 4 concurrent real-time tasks
  - Semaphores for resource management
  - Message queues for inter-task communication
  - Critical sections for data protection

- ✅ **Energy Monitoring**
  - 1 kHz ADC sampling rate (hardware-triggered)
  - Real-time current measurement (0-20A range)
  - Current-only telemetry (voltage/power calculated on web side)

- ✅ **Relay Control**
  - 2 independent relay channels
  - Manual control via physical buttons
  - Remote control via web dashboard
  - Automatic overcurrent protection

- ✅ **IoT Integration**
  - STM32 → ESP32 via UART (115200 baud)
  - ESP32 → Firebase Real-time Database
  - Firebase → Web Dashboard (real-time updates)
  - Bidirectional command/control flow

- ✅ **Web Dashboard**
  - Real-time current monitoring
  - Historical data with date filtering
  - Interactive relay toggles
  - Responsive Chart.js visualizations

---

## 🏗️ System Architecture

### Data Flow

**Monitoring Flow (STM32 → Web):**
```
1. ACS712 Sensor → STM32 ADC (1 kHz sampling)
2. SensorTask processes ADC → calculates current
3. UartTxTask sends JSON every 1 second:
   {"current":2.350,"relay1":1,"relay2":0,"time":12345}
4. ESP32 receives JSON via UART2
5. ESP32 uploads to Firebase:
   /RelayData1/{timestamp} = current_value
   /RelayData2/{timestamp} = current_value
6. Web dashboard listens to Firebase
7. Chart.js updates graphs in real-time
```

**Control Flow (Web → STM32):**
```
1. User clicks relay toggle on website
2. JavaScript updates Firebase:
   /relayToggle1 = true/false
3. ESP32 polls Firebase every 300ms
4. ESP32 detects change, sends UART command:
   "L1_ON\n" or "L1_OFF\n"
5. STM32 ControlTask receives command
6. STM32 controls physical relay
```

---

## 🛠️ Hardware Requirements

### Main Components

| Component | Model | Quantity | Purpose |
|-----------|-------|----------|---------|
| **Microcontroller** | STM32 Nucleo-F401RE | 1 | Main control & RTOS |
| **WiFi Module** | ESP32-WROOM-32 | 1 | IoT gateway |
| **Current Sensor** | ACS712-20A | 1 | AC current measurement |
| **Relays** | 12V SPDT Relay | 2 | Load switching |
| **Shift Register** | SN74LV8153 | 1 | Relay driver |
| **Buttons** | Tactile Push Button | 3 | Manual control |
| **LED** | Standard LED | 1 | Status indicator |


**Power Requirements:**
- STM32: 5V via USB or external
- ESP32: 3.3V (onboard regulator) or 5V USB
- Relays: 12V external supply
- ACS712: 5V

---

## 💻 Software Requirements

### Development Tools

**For STM32:**
- STM32CubeIDE v1.19.0 or later
- STM32CubeMX (integrated)
- ARM GCC Toolchain (bundled)

**For ESP32:**
- Visual Studio Code
- PlatformIO Extension
- Arduino Framework for ESP32

**For Web Dashboard:**
- Firebase account

### Required Libraries

**STM32 (HAL Drivers):**
- STM32F4xx HAL Driver
- CMSIS
- Custom RTOS

---

## 🧠 Custom RTOS Implementation

### Overview

The custom RTOS is a **priority-based cooperative scheduler** designed for real-time energy monitoring applications.

**Key Features:**
- ✅ No external dependencies (built from scratch)
- ✅ Priority-based task scheduling (0 = highest)
- ✅ Cooperative multitasking (tasks yield voluntarily)
- ✅ SysTick-based timing (1ms tick)
- ✅ Semaphores for mutual exclusion
- ✅ Message queues for inter-task communication
- ✅ Critical sections for atomic operations

### Task Architecture

| Task | Period | Priority | WCET | Purpose |
|------|--------|----------|------|---------|
| **SensorTask** | 10ms | 0 (highest) | ~500µs | ADC sampling & current calculation |
| **ControlTask** | 20ms | 1 | ~300µs | Button handling & relay control |
| **UartTxTask** | 1000ms | 2 | ~2ms | JSON telemetry transmission |
| **HeartbeatTask** | 500ms | 3 (lowest) | ~10µs | LED status indicator |

### CPU Utilization

```
Total CPU Usage: ~7%
Idle Time: ~93%
Real-time guarantees: All tasks meet deadlines
```

---

## 🔌 Hardware Setup

### Step 1: Wire ACS712 Sensor

```
ACS712 Pin    →  STM32 Pin
─────────────────────────────
VCC           →  5V
GND           →  GND
OUT           →  PA0 (ADC1_CH0)
```

**AC Load Connection:**
- Connect AC hot wire through ACS712 screw terminals
- ACS712 measures AC current up to 20A
- Output voltage: 2.5V at 0A, ±0.1V per Ampere

### Step 2: Wire Relays

```
STM32 Pin   →  Relay Module
──────────────────────────────
PB0         →  Relay 1 IN
PB1         →  Relay 2 IN
5V          →  VCC
GND         →  GND
```

**Relay Connections:**
- COM: Common (from power source)
- NO: Normally Open (to load)
- NC: Normally Closed (unused)

### Step 3: Wire Buttons

```
Button      STM32 Pin    Configuration
────────────────────────────────────────
BTN_ALL  →  PC0        Pull-up, active-low
BTN_L1   →  PC1        Pull-up, active-low
BTN_L2   →  PC2        Pull-up, active-low
```

### Step 4: Connect ESP32

```
ESP32 GPIO16 (RX2)  →  STM32 PA2 (TX)
ESP32 GPIO17 (TX2)  →  STM32 PA3 (RX)
ESP32 GND           →  STM32 GND
```

**⚠️ Important:** Ensure common ground between STM32 and ESP32!

---

## ⚙️ Software Configuration

### STM32 Configuration (CubeMX)

**ADC1 Settings:**
```
Channel: ADC_CHANNEL_0 (PA0)
Resolution: 12-bit (0-4095)
Trigger: Timer 2 TRGO (1 kHz)
Mode: External trigger, interrupt
```

**Timer 2 Settings:**
```
Prescaler: 84-1
Period: 1000-1
Clock: 84 MHz / 84 / 1000 = 1 kHz
Trigger Output: Update Event (TRGO)
```

**UART2 Settings (ESP32):**
```
Baud Rate: 115200
Word Length: 8 bits
Stop Bits: 1
Parity: None
Mode: TX/RX
```

**GPIO Configuration:**
```
PA0: ADC1_CH0 (Analog Input)
PA2: USART2_TX
PA3: USART2_RX
PA5: GPIO_Output (LD2)
PB0: GPIO_Output (Relay1)
PB1: GPIO_Output (Relay2)
PC0-PC2: GPIO_Input (Buttons, Pull-up)
```

---

## 🌐 Web Dashboard

### Features

- **Real-time Charts:** Two Chart.js line graphs showing current over time
- **Date Filtering:** Select specific dates to view historical data
- **Relay Control:** Toggle switches for remote relay control
- **Responsive Design:** Works on desktop and mobile

### File Structure

```
Web/
├── index.html       # Main HTML structure
├── line_chart.js    # Firebase + Chart.js logic
└── project.css      # Styling
```

### Running Locally

**Option 1: Simple HTTP Server (Python)**
```bash
cd Web/
python -m http.server 8000
# Visit: http://localhost:8000
```

**Option 2: Live Server (VSCode)**
1. Install "Live Server" extension
2. Right-click `index.html`
3. Select "Open with Live Server"

**Option 3: Direct File**
- Simply open `index.html` in browser
- May have CORS issues with Firebase

### Hosting Options

**Firebase Hosting (Recommended):**
```bash
npm install -g firebase-tools
firebase login
firebase init hosting
firebase deploy
```

**GitHub Pages:**
1. Push code to GitHub
2. Settings → Pages → Deploy from main branch
3. Access at: `https://username.github.io/repo-name`

---

## 🧪 Testing & Validation

### Performance Validation

| Metric | Requirement | Measured | Status |
|--------|-------------|----------|--------|
| ADC Sampling Rate | ≥1 kHz | 1000.2 Hz | ✓ Pass |
| SensorTask Period | 10ms | 10.0±0.1ms | ✓ Pass |
| UART Data Rate | 1 Hz | 1.00 Hz | ✓ Pass |
| CPU Utilization | <50% | ~7% | ✓ Pass |
| Power Consumption | <2W | 1.2W | ✓ Pass |
| Current Accuracy | ±5% | ±1% | ✓ Pass |

---

## 🐛 Troubleshooting

### STM32 Issues

**Problem: LD2 Not Blinking**
```
Cause: RTOS not running or SysTick not configured
Fix:
1. Check SysTick_Handler calls RTOS_Tick()
2. Verify RTOS_Init(1) called in main()
3. Check while(1) loop calls RTOS_RunScheduler()
```

**Problem: No UART Output**
```
Cause: UART not initialized or wrong baud rate
Fix:
1. Check UART2 initialized: MX_USART2_UART_Init()
2. Verify baud rate = 115200
3. Check TX pin (PA2) connected to ESP32 RX
```

**Problem: ADC Always Reads 0**
```
Cause: Timer2 not started or ADC not triggered
Fix:
1. Add HAL_TIM_Base_Start(&htim2);
2. Add HAL_ADC_Start_IT(&hadc1);
3. Check ADC callback: HAL_ADC_ConvCpltCallback()
```

### ESP32 Issues

**Problem: WiFi Won't Connect**
```
Cause: Wrong SSID/password or network issues
Fix:
1. Double-check WIFI_SSID and WIFI_PASSWORD
2. Try 2.4GHz network (ESP32 doesn't support 5GHz)
3. Check serial monitor for error messages
```

**Problem: Firebase Upload Fails**
```
Cause: Wrong API key or database URL
Fix:
1. Verify API_KEY matches Firebase Console
2. Check DATABASE_URL format: https://xxx.firebaseio.com/
3. Ensure Firebase rules allow write access
```

**Problem: No Data from STM32**
```
Cause: UART not receiving or wrong pins
Fix:
1. Check RXD2 (GPIO16) connected to STM32 TX (PA2)
2. Verify baud rate = 115200 on both sides
3. Test with Serial2.println("test"); on STM32
```

### Web Dashboard Issues

**Problem: Charts Not Loading**
```
Cause: Firebase config wrong or CORS issues
Fix:
1. Check firebaseConfig in line_chart.js
2. Open browser console (F12) for errors
3. Try hosting via HTTP server instead of file://
```

**Problem: Relay Toggles Don't Work**
```
Cause: Firebase listener not attached
Fix:
1. Check browser console for errors
2. Verify toggle IDs match: relayToggle1, relayToggle2
3. Check Firebase rules allow write access
```

---

## 🎓 Academic Information

**Course:** Embedded Systems Capstone Project  
**Institution:** Texas Tech University

**Team Members:**
- **Austin Alaspa** - Hardware Design & PCB Layout
- **Davis Crose** - ESP32 & Firebase Integration
- **Edwin Sanchez** - Web Dashboard & Testing

---

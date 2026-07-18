# EdgeAI-Fall-Sentinel

# Fall-Detection-IoT-STM32

**Real-time Fall Detection System using Embedded Microcontroller (Edge-AI) and Dual-Communication Architecture**

This is the open-source repository for a graduation thesis by students from the Faculty of Information Technology. The project focuses on solving the fall detection problem for the elderly and workers in hazardous environments by embedding a Machine Learning model directly onto a resource-constrained microcontroller.

---

## Key Features

* **Edge-AI Verifier (Hybrid Algorithm):** Combines a Finite State Machine (FSM) as a pre-filter with a Support Vector Machine (SVM) model extracting 9 kinematic features. Achieves an **F1-score of 94%** on the internal test dataset.
* **DSP Optimization (Digital Signal Processing):** Automatically configures the sensor to the maximum measurement range (±16g, ±2000°/s) to prevent signal saturation (clipping). Integrates a decimation and low-pass filter averaging 20 samples to downsample the frequency from 100Hz to 5Hz, ensuring absolute synchronization with the SisFall dataset.
* **Dual-Communication (Dual-Comm):** Parallel alerting via cellular network (SMS - Internet independent) and IoT protocol (MQTT - QoS 1) with a UART collision avoidance mechanism.
* **Low-power Consumption:** The device operates on an 8MHz HSI internal oscillator, maintaining an average system current consumption of ~17mA. Achieves an estimated runtime of ~63 hours of continuous operation with a 1200mAh LiPo battery.
* **OTA Configuration (Over-the-Air):** Updates the emergency phone number directly via MQTT and permanently stores it in the Flash ROM (Page 62).

---

## Hardware Architecture

The system is designed with a 3-layer stacked architecture optimized for a waist-mounted wearable:

* **MCU:** STM32F103C8T6 (ARM Cortex-M3).
* **Sensor:** MPU6050 (6-DoF Accelerometer & Gyroscope) communicating via I2C.
* **Telecommunication:** SIM7680C (4G LTE Cat.1 & GNSS/GPS) communicating via USART2 with an isolated power supply mechanism handling 2A peak current.
* **UI/UX Interface:** Buzzer (Pre-alert siren) & Hardware SOS Button (Activate/Cancel alarm).

---

## Flash configuration reservation

The STM32F103C8T6 application Flash region is limited to 62 KB
in `firmware/STM32F103C8TX_FLASH.ld`.

The address range `0x0800F800–0x0800FFFF` is reserved for
persistent configuration data. The emergency phone number is
stored beginning at `0x0800F800`.

The linker map generated from the final firmware build is available
at `firmware/build-evidence/FallDetection.map`.

---

## Repository Structure

To ensure transparency and reproducibility in accordance with scientific evaluation requirements, this repository contains all system components:

* `/firmware_stm32/`: C/C++ source code flashed to the STM32 microcontroller (Implements the 100Hz Timer interrupt routine, ultra-lightweight linear SVM algorithm with < 1ms execution time, and AT command management).
* `/ai_training_artifacts/`: (Directory for Dataset and Python Scripts)
* Jupyter Notebook file for processing the SisFall dataset.
* Script for feature extraction and SVM model training (Hyperparameter tuning and determination of the decision threshold $\tau = 1.102146$).
* `/android_app/`: Source code for the monitoring application on the Android operating system (Kotlin). Communicates with the device via EMQX Broker and stores event logs in Firebase.

---

## Limitations & Future Work

1. **OTA Security:** The function to receive phone numbers via MQTT is currently operating at a prototype level. Future iterations require integrating TLS/SSL certificates on the EMQX Broker and authenticating the JSON format to establish a Trust Boundary.
2. **Circuit Size:** The current version uses through-hole components for testing purposes. The physical footprint can be reduced by designing a PCB with surface-mount devices (SMD).

---

## Authors

* **Performed by students:**
* Bui Tan Dat
* Ha Huu Thinh

# EdgeAI Fall Sentinel

**Wearable fall-detection prototype using hybrid FSM--SVM verification on STM32 and cellular SMS/MQTT alerting**

EdgeAI Fall Sentinel is a graduation-thesis prototype developed at the Faculty of Information Technology, Ho Chi Minh City University of Foreign Languages and Information Technology (HUFLIT). The system performs fall verification locally on an STM32F103C8T6 and uses a SIM7680C development module for cellular notification.

> **Research prototype:** This project is not a certified medical device and must not be used as the sole emergency-response mechanism.

## System overview

- **Hybrid edge inference:** a Finite State Machine (FSM) gates candidate impacts before a 9-feature linear Support Vector Machine (SVM) performs verification.
- **Real-time sensing:** valid IMU samples are processed at 100 Hz and block-averaged in groups of 20 to obtain a 5 Hz AI representation.
- **Bounded acquisition:** the nominal post-impact AI window contains 25 points over 5 seconds, with a 7-second safety timeout when invalid samples delay collection.
- **Cooperative modem handling:** timer-driven sensor work, UART servicing, emergency cancellation and watchdog refresh continue during bounded AT-command waits.
- **Local and remote notification:** the controlled prototype supports a local buzzer, SMS and MQTT. Plaintext MQTT is disabled by default in the public firmware build.
- **Persistent configuration:** the emergency phone-number record is placed in a linker-reserved Flash region beginning at `0x0800F800`.

## Final offline model

The final learning pipeline uses mutually exclusive subject-level training, validation and test cohorts containing 22, 8 and 8 SisFall subjects, respectively. The scaler and linear SVM are fitted only on the training cohort. The decision threshold is selected only from validation scores to satisfy recall >= 0.95, while the test cohort remains sequestered until final evaluation.

Deployed parameters:

| Parameter | Value |
| --- | ---: |
| SVM decision threshold `tau` | `0.298502437` |
| SVM bias | `-2.114027035` |
| Number of features | 9 |
| Training SVM configuration | Linear kernel, `C=1.0`, balanced class weights |

Final subject-independent offline test confusion matrix, using the convention `[[TN, FP], [FN, TP]]`:

```text
[[2940, 42],
 [  11, 502]]
```

| Metric | Value |
| --- | ---: |
| Accuracy | 98.48% |
| Precision | 92.28% |
| Recall / Sensitivity | 97.86% |
| Specificity | 98.59% |
| F1-score | 94.99% |
| False-positive rate | 1.41% |
| False-negative rate | 2.14% |

These are **window-level offline classifier results**, not end-to-end accuracy of the complete wearable. The final exported model has not yet been re-evaluated in the previous 100-trial on-device pilot. Multiple windows may originate from the same subject, so simple binomial confidence intervals must not be interpreted as subject-level uncertainty.

## Hardware

- **MCU:** STM32F103C8T6, ARM Cortex-M3, running at 36 MHz from the 8 MHz HSI divided by two and multiplied by nine through the PLL.
- **IMU:** six-axis module compatible with the MPU60x0/MPU65x0 register map, connected over I2C and configured for +/-16 g and +/-2000 degrees/s. The tested unit returned `WHO_AM_I = 0x70`; it is therefore not claimed to be a confirmed original MPU6050.
- **Cellular/GNSS:** SIM7680C development module connected through USART2.
- **User interface:** local buzzer and hardware SOS/cancel button.
- **Power mitigation:** a 2200 uF low-ESR capacitor is placed close to the 5 V VDD/USB input of the SIM7680C development module to reduce LTE current-transient effects.

Battery runtime and average current have **not** been validated using a precision power analyzer. This repository therefore does not claim a measured 17 mA average current or a verified 63-hour runtime. Direct rail-level waveforms, BOR option-byte verification and battery-side energy logging remain future characterization work.

## Security defaults

The distributed source is configured to fail closed:

```c
#define APP_ENABLE_PLAINTEXT_MQTT        0
#define APP_ALLOW_REMOTE_PHONE_CONFIG    0
```

Consequently, plaintext MQTT publication and remote caregiver-number updates are not enabled without an explicit deployment decision. These defaults reduce accidental disclosure but do **not** implement TLS in the STM32 firmware.

Before enabling remote communication in a real deployment, provide:

- certificate-validated TLS supported by the modem and broker;
- broker authentication and per-device topic authorization;
- private, deployment-specific endpoints and credentials;
- integrity-protected, versioned configuration records;
- a private emergency phone number supplied outside source control.

The Android project externalizes deployment values through `secrets.properties`; start from `secrets.properties.example` and never commit the completed secrets file. Firebase rules included in the application archive must be reviewed and deployed for the intended project.

## Flash configuration reservation

The STM32F103C8T6 application image is limited to the first 62 KB of Flash:

```ld
FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 62K
```

The range `0x0800F800`--`0x0800FFFF` is reserved for persistent configuration, with the phone-number record beginning at `0x0800F800`. The linker script must enforce this boundary; a C constant alone is insufficient. Before releasing a firmware binary, retain the generated `.map` file and verify that no application section reaches the reserved page.

## Repository contents

The current repository distributes the implementation as archives:

```text
EdgeAI-Fall-Sentinel/
|-- FallDetectionSTM32.zip   # STM32CubeIDE firmware project
|-- FallDetectionApp.zip     # Android/Kotlin application project
`-- README.md
```

Extract the archives before building or reviewing the source. For better reproducibility, future releases should also expose browsable `firmware/`, `android_app/` and `ai_training/` directories rather than relying only on ZIP files.

The exact final training notebook/script, subject-split manifest, Python environment lockfile and machine-readable evaluation outputs are not included in the current repository revision. They must be added before the repository can be considered a complete independent-reproduction package.

## Build notes

### Firmware

1. Extract `FallDetectionSTM32.zip`.
2. Import the project into STM32CubeIDE.
3. Keep deployment-specific values in the private application configuration header; do not hard-code real phone numbers or credentials in tracked source.
4. Confirm that the active linker script contains `FLASH LENGTH = 62K`.
5. Build the project and inspect the generated `.map` file to ensure the application does not overlap `0x0800F800`.

### Android application

1. Extract `FallDetectionApp.zip` and open the project in Android Studio.
2. Copy `secrets.properties.example` to `secrets.properties`.
3. Configure a private TLS-capable broker and the intended Firebase project without committing secrets.
4. Review and deploy the supplied Firebase Realtime Database and Firestore rules.
5. Build and test the application on a controlled account before connecting it to a device.

## Known limitations

1. **Training/deployment window mismatch:** offline fall windows are peak-centered, whereas the firmware collects a predominantly post-impact window. A pre-trigger circular buffer or retraining with firmware-equivalent windows is required.
2. **Final on-device validation pending:** the earlier five-subject, 100-trial pilot used an older scaler, classifier, bias and threshold. Its 94% F1-score must not be attributed to the final model.
3. **Communication security incomplete:** the STM32 release fails closed but does not yet provide certificate-validated MQTT TLS.
4. **Timing characterization pending:** worst-case sampling deadlines, UART overflow/desynchronization and end-to-end GNSS/cellular latency distributions have not been measured systematically.
5. **Power characterization pending:** battery-side current, energy and runtime require precision logging across standby, telemetry and alarm scenarios.
6. **External validity limited:** broader evaluation with target users, varied body placements, repeated grouped splits and subject-level reporting is required.

## Citation

Repository: <https://github.com/huuthinh2005/EdgeAI-Fall-Sentinel>

Revision used by the current paper draft: `6c53aa865f0f04c614a18b36cb7e3b4f62e002a0` (July 18, 2026).

If the repository is updated, update the revision cited in the paper before submission.

## Authors

- Bui Tan Dat
- Ha Huu Thinh

Supervisors:

- Pham Cong Thien
- Ho Le Minh Toan

## License status

No project-level license has been selected in the current repository revision. The source is publicly available for academic review, but reuse and redistribution permissions are not granted until a `LICENSE` file is added. Do not describe the project as open source until an explicit license is published.

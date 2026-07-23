EdgeAI Fall Sentinel

Wearable fall-detection prototype using hybrid FSM--SVM verification on STM32 and dual-channel SMS/MQTT alerting

EdgeAI Fall Sentinel is a graduation-thesis prototype developed at the Faculty of Information Technology, Ho Chi Minh City University of Foreign Languages and Information Technology (HUFLIT). The wearable performs candidate gating and fall verification locally on an STM32F103C8T6, while a SIM7680C development module provides GNSS, SMS, and MQTT connectivity. An Android application receives telemetry and alarm messages and stores selected records in Firebase.

Research prototype: This system is not a certified medical device and must not be used as the sole emergency-response mechanism.

System overview

Hybrid edge inference: a finite-state machine (FSM) screens candidate events before a nine-feature linear support-vector machine (SVM) performs verification.

Continuous sensing: valid IMU samples are processed at 100 Hz.

Bounded AI acquisition: groups of 20 valid 100-Hz samples are averaged to form one 5-Hz point. A nominal AI window contains 25 points over 5 s, with a 7-s safety timeout.

Cooperative modem handling: sensing, UART reception, alarm cancellation, and watchdog servicing continue during bounded AT-command waits.

Local and remote notification: an SVM-confirmed event activates the local warning and starts a 10-s cancellation interval. If it is not cancelled, the firmware queries GNSS, sends an emergency SMS, and publishes an MQTT alarm.

Adaptive telemetry: telemetry is scheduled every 5 min while moving and every 30 min while still.

Persistent configuration: the caregiver phone-number record begins at 0x0800F800, outside the linker-limited application region.

Final offline model

The final learning pipeline uses mutually exclusive subject-level training, validation, and test cohorts from SisFall:

Cohort

Subjects

Windows

Training

22

13,406

Validation

8

4,719

Test

8

3,495

Total

38

21,620

The StandardScaler and linear SVM are fitted only on training subjects. The threshold is selected only from validation scores to satisfy recall >= 0.95; the test cohort remains sequestered until final evaluation.

Deployed parameters:

Parameter

Value

SVM decision threshold tau

0.298502437

SVM bias

-2.114027035

Number of features

9

Classifier

Linear SVM, C=1.0, balanced class weights

Final subject-independent offline test confusion matrix using [[TN, FP], [FN, TP]]:

[[2940, 42],
 [  11, 502]]

Metric

Value

Accuracy

98.48%

Precision

92.28%

Recall / sensitivity

97.86%

Specificity

98.59%

F1-score

94.99%

False-positive rate

1.41%

False-negative rate

2.14%

These are window-level offline SVM results. They do not measure the end-to-end accuracy of the complete wearable, the FSM gate, user cancellation, or cellular delivery. Multiple windows may originate from one subject, so simple window-level confidence intervals must not be interpreted as subject-level uncertainty.

Training and deployment alignment

The offline and embedded pipelines intentionally share the same nine-feature order, scaling constants, SVM coefficients, bias, threshold, and tilt expression. The exported parameters in result/svm_model_generated.txt match the constants in the current firmware.

A remaining limitation is window construction:

SisFall recordings are sampled at 200 Hz and averaged in groups of 40 to obtain 5-Hz points.

Training fall windows are peak-centered.

Firmware samples at 100 Hz and averages groups of 20 valid samples.

The embedded window begins with the first confirmed impact in its first averaging bucket and is predominantly post-impact.

Therefore, the offline metrics characterize the held-out SVM windows and must not be presented as final deployed or clinical performance. A pre-trigger circular buffer or retraining with firmware-equivalent windows is required for stronger deployment claims.

Hardware

MCU: STM32F103C8T6, ARM Cortex-M3, operated at 36 MHz.

IMU: six-axis module compatible with the MPU60x0/MPU65x0 register map, connected over I2C and configured for +/-16 g and +/-2000 deg/s. The tested unit returned WHO_AM_I = 0x70; it is not claimed to be a confirmed original MPU6050.

Cellular/GNSS: SIM7680C development module connected through USART2.

Local interface: buzzer/LED outputs and a hardware SOS/cancel button.

Power path: a TP4056 module charges the Li-Po cell, while an MT3608 supplies 5 V to the VDD/USB input of the SIM7680C development board.

Transient mitigation: a 2200-uF low-ESR capacitor is placed close to the SIM7680C board's 5-V input.

Battery runtime, average current, direct MCU-rail transient waveforms, and POR/PDR/PVD margin have not been characterized using dedicated measurement equipment.

Communication and security status

The integrated firmware currently builds with:

#define ENABLE_SIM 1

For controlled prototype testing, it connects to:

tcp://broker.emqx.io:1883

and uses the following MQTT topics:

thiet_bi/cai_dat
thiet_bi/telemetry
thiet_bi/bao_dong

The current MQTT path is unauthenticated and unencrypted. It is suitable only for controlled prototype demonstrations. Production deployment would require certificate-validated TLS, broker authentication, per-device topic authorization, private deployment-specific identifiers, and integrity-protected versioned configuration records.

Do not commit private emergency phone numbers, Firebase service-account credentials, keystores, completed deployment secrets, or other personal data.

Flash Page 62 reservation

The linker script limits the application to the first 62 KiB of Flash:

FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 62K

Persistent configuration begins at 0x0800F800. In the retained final linker map:

the .data load image begins at 0x0800D638;

the .data load image ends at 0x0800D81C;

the remaining margin before 0x0800F800 is 0x1FE4, or 8,164 bytes.

Relevant evidence:

firmware/Fall_Detection_STM32/STM32F103C8TX_FLASH.ld
firmware/Fall_Detection_STM32/Debug/falldetection_real5.map

The project must be rebuilt and the map rechecked whenever firmware changes.

Repository contents

EdgeAI-Fall-Sentinel/
|-- androidapp/
|   `-- FallDetectionApp/App/          # Android/Kotlin project
|-- firmware/
|   `-- Fall_Detection_STM32/          # STM32CubeIDE project
|-- training/
|   `-- TrainDetectionAI/
|       `-- TrainAIDetection.ipynb     # Subject-independent training notebook
|-- result/
|   |-- svm_model_generated.txt        # Exported scaler/SVM constants
|   `-- svm_training_results.json      # Split, window counts, and metrics
`-- README.md

The raw SisFall dataset is not redistributed. Download it from the dataset source cited in the paper and place the converted CSV files in the training data directory expected by the notebook.

Reproducibility artifacts

result/svm_training_results.json records:

random seed and target recall;

processed-file and total-window counts;

exact training, validation, and test subject lists;

training, validation, and test window counts;

validation threshold and validation metrics;

test confusion matrix, classification report, and specificity.

result/svm_model_generated.txt contains the exact exported constants used by the firmware.

Build notes

Firmware

Open or import firmware/Fall_Detection_STM32 in STM32CubeIDE.

Confirm that the active linker script is STM32F103C8TX_FLASH.ld.

Rebuild the project.

Inspect the generated map and verify that no Flash load section reaches 0x0800F800.

Keep deployment-specific phone numbers and credentials outside source control.

Android application

Open androidapp/FallDetectionApp/App in Android Studio.

Supply the intended Firebase configuration locally; do not commit service-account credentials.

Review Firebase Authentication, Realtime Database, and Firestore rules for the intended test project.

Build and test with controlled accounts and non-sensitive prototype data.

Model training

Open training/TrainDetectionAI/TrainAIDetection.ipynb.

Provide the SisFall CSV directory expected by the notebook.

Run all cells to reproduce the subject split, model export, and evaluation JSON.

Confirm that the generated constants match firmware/Fall_Detection_STM32/Core/Src/main.c.

Known limitations

Training/deployment window mismatch: training uses peak-centered windows, while firmware uses a predominantly post-impact window.

Final-model device validation pending: the exact deployed model has not undergone a controlled, quantitative on-device replay or equivalence study.

Communication security incomplete: the current public-broker MQTT path uses plaintext port 1883 without authentication.

Timing characterization pending: worst-case sampling deadlines, UART overflow/desynchronization, and end-to-end GNSS/cellular latency distributions have not been measured systematically.

Power characterization pending: battery-side current, energy, runtime, and MCU-rail transient margin require dedicated measurement.

External validity limited: broader evaluation with target users, varied placements, repeated grouped splits, and subject-level uncertainty reporting is required.

Data and ethics statement

Quantitative machine-learning development and evaluation use only the public, de-identified SisFall dataset. No additional human-participant dataset was collected. The prototype photograph documents a voluntary author-performed qualitative engineering self-demonstration and was not used for model training or quantitative evaluation.

Citation and versioning

Repository:

https://github.com/huuthinh2005/EdgeAI-Fall-Sentinel

For a manuscript or thesis, cite an immutable commit URL rather than the moving main branch. After the final repository update, copy the resulting commit SHA into the paper's Data Availability section.

Authors

Bui Tan Dat

Ha Huu Thinh

Supervisors:

Ho Le Minh Toan

Pham Cong Thien

License status

No project-level license has been selected. The source is publicly available for academic review, but reuse and redistribution permissions are not granted until a LICENSE file is added. Do not describe the repository as open source unless an explicit license is published.

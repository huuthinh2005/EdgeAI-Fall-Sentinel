/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Fall Detection with Real MPU6050 + Panic Button (Pro Version)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	MPU_SAMPLE_NONE = 0,
	MPU_SAMPLE_VALID,
	MPU_SAMPLE_INVALID,
	MPU_SAMPLE_LOW_G_PENDING
} MPU_Sample_Result_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Linker MEMORY phải giới hạn FLASH ứng dụng còn 62K; vùng từ địa chỉ này
 * trở đi được dành riêng cho dữ liệu cấu hình. */
#define FLASH_ADDR_PHONE  0x0800F800U  // Lưu SĐT ở Page 62

// Địa chỉ I2C của MPU6050
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define ALARM_COOLDOWN_MS 30000
#define I2C_RECOVERY_MIN_INTERVAL_MS 1000U
#define MPU_DMA_FRAME_SIZE 14U
#define MPU_DMA_TIMEOUT_MS 20U
#define MPU_DMA_SOFT_RETRY_MS 10U
#define MPU_LOW_G_CONFIRM_THRESHOLD 0.10f
#define IMPACT_WAIT_TIMEOUT_MS 1000
#define AI_COLLECTION_TIMEOUT_MS 7000
#define SENSOR_SAMPLE_PERIOD_MS 10U
#define AI_BUCKET_SAMPLES 20U
#define AI_FEATURE_COUNT 9U
#define SENSOR_CLEAN_ARM_SAMPLES 200U
#define IMPACT_CONFIRM_SAMPLES 2U
#define IMPACT_CONFIRM_MAX_GAP_MS 30U
#define AI_MAX_INVALID_SAMPLES 3U
#define MPU_RAW_SATURATION_LIMIT 32700
#define UART_PROCESS_BUDGET 256U
#define PHONE_FLASH_MIN_INTERVAL_MS 3600000UL
#define RAD_TO_DEG_F 57.2957795131f

/* Bản tích hợp cuối: bật UART2, Network, GNSS, MQTT và SMS. RF di động được
 * giữ hoạt động sau khởi động; không dùng CFUN=4/CFUN=1 trong runtime. */
#ifndef ENABLE_SIM
#define ENABLE_SIM 1
#endif

/* Thời gian nhận retained config sau khi MQTT subscribe. */
#define SIM_CONFIG_RX_WINDOW_MS 1000U

/* Bản chạy tích hợp không in Raw mỗi 2 giây để giảm UART blocking/Miss.
 * Có thể đổi thành 1 khi cần chẩn đoán chi tiết cảm biến. */
#ifndef ENABLE_VERBOSE_DEBUG
#define ENABLE_VERBOSE_DEBUG 0
#endif

#ifndef ENABLE_AT_TRACE
#define ENABLE_AT_TRACE 0
#endif

#ifndef ENABLE_HEALTH_LOG
#define ENABLE_HEALTH_LOG 0
#endif

/* Bản final không cấp RAM và thời gian UART cho Ring Buffer chẩn đoán. */
#ifndef ENABLE_RING_LOG
#define ENABLE_RING_LOG 0
#endif

// Các ngưỡng phát hiện té ngã
#define FALL_THRESHOLD 2.5       // Ngưỡng va chạm mạnh (2.5g)
#define STABLE_THRESHOLD_MIN 0.8 // Ngưỡng dưới trạng thái nằm im
#define STABLE_THRESHOLD_MAX 1.2 // Ngưỡng trên trạng thái nằm im
#define FREE_FALL_THRESHOLD 0.6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DMA_HandleTypeDef hdma_i2c1_rx;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char phone_number[20] = "";
char pending_phone_number[20] = "";
volatile uint8_t new_phone_pending_flag = 0;

volatile float latitude = 0, longitude = 0;
volatile int16_t Accel_X_RAW = 0, Accel_Y_RAW = 0, Accel_Z_RAW = 0;
volatile float Ax, Ay, Az;
volatile float SVM_raw = 0.0f;     // FIX 1: Dùng bắt va chạm (Không lọc)
volatile float SVM_smooth = 1.0f;  // FIX 1: Dùng tracking hành vi (Đã lọc)

volatile uint8_t mpu_read_flag = 0;
volatile uint8_t fall_detected_flag = 0;
volatile uint8_t gps_valid = 0;
volatile uint8_t first_alarm = 1;
volatile uint8_t mpu_ok = 0;
volatile uint8_t fall_state = 0;
volatile uint8_t mqtt_connected = 0;
volatile uint8_t panic_button_flag = 0;

// Khai báo Ring Buffer UART2
#define RX3_BUF_SIZE 1024
volatile uint8_t rx3_buf[RX3_BUF_SIZE];
volatile uint16_t rx3_head = 0;
volatile uint16_t rx3_tail = 0;
uint8_t rx3_byte;
volatile uint32_t rx3_overflow_count = 0;
volatile uint8_t rx3_desync = 0;
/* Nếu ISR không thể arm lại RX, main loop sẽ abort/re-arm an toàn. */
volatile uint8_t uart2_restart_pending = 0;
/* 1 khi một hàm AT đang trực tiếp đọc ring RX; parser nền không được lấy byte. */
volatile uint8_t sim_at_transaction_active = 0;
uint32_t uart_line_overflow_count = 0;

/* Parser UART2 phải được khai báo trước Flush_UART2_Buffer(). */
static char process_buf[256];
static uint16_t process_idx = 0;
static uint8_t expecting_phone = 0;
static uint8_t process_drop_line = 0;

volatile uint8_t is_moving = 0;
uint8_t battery_percent = 85;
uint32_t telemetry_interval = 1800000;
uint32_t last_telemetry_tick = 0;
uint32_t motion_stable_count = 0;
uint32_t motion_active_count = 0;
volatile uint8_t alarm_transmit_active = 0;

// Biến cho còi báo động
uint8_t pre_alarm_active = 0;
uint32_t pre_alarm_start_time = 0;
char alarm_reason_str[64] = "";

// =======================================================
// BIẾN THU THẬP DATA TRAIN AI (GOM BATCH 5 GIÂY)
// =======================================================
volatile int16_t Gyro_X_RAW = 0, Gyro_Y_RAW = 0, Gyro_Z_RAW = 0;
volatile float Gx = 0, Gy = 0, Gz = 0;

// Cấu trúc 1 mẫu dữ liệu AI
typedef struct {
	float ax, ay, az;
	float gx, gy, gz;
} AI_Sample_t;

#define AI_BATCH_SIZE 25          // Gom 25 mẫu (Tần số 5Hz trong 5 giây)
AI_Sample_t ai_buffer[AI_BATCH_SIZE];
uint8_t ai_sample_count = 0;
volatile uint8_t mpu_sample_valid = 0;
volatile uint8_t mpu_sample_saturated = 0;
volatile uint8_t mpu_saturation_observed = 0;
volatile uint8_t mpu_sensor_armed = 0;
volatile uint16_t mpu_clean_sample_streak = 0;
volatile uint8_t mpu_dma_busy = 0;
volatile uint8_t mpu_dma_frame_ready = 0;
volatile uint8_t mpu_dma_error_pending = 0;
volatile uint8_t mpu_recovery_active = 0;
volatile uint8_t mpu_dma_rx[MPU_DMA_FRAME_SIZE];
volatile uint32_t mpu_dma_start_tick = 0;
volatile uint32_t mpu_dma_retry_after_tick = 0;
volatile uint32_t mpu_dma_error_code = HAL_I2C_ERROR_NONE;
volatile uint32_t mpu_last_hal_error = HAL_I2C_ERROR_NONE;
volatile uint32_t mpu_missed_deadline_count = 0;
volatile uint32_t mpu_read_failure_count = 0;
volatile uint32_t mpu_rejected_saturation_count = 0;
volatile uint32_t mpu_low_g_candidate_count = 0;
volatile uint32_t i2c_recovery_line_fault_count = 0;
volatile uint8_t mpu_who_am_i = 0;
volatile uint8_t mpu_init_stage = 0;
volatile uint8_t mpu_init_readback = 0;
uint32_t fall_state_enter_tick = 0;
uint32_t ai_collection_start_tick = 0;
uint32_t last_phone_flash_write_tick = 0;
uint8_t phone_flash_written_this_boot = 0;
volatile float ai_last_score = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t Read_Battery_Percent(void);
void Send_Emergency_SMS(char *reason);
void SIM7680_Network_Init(void);
void SIM7680_GNSS_Init(void);
void SIM7680_MQTT_Init(void);
void Update_GPS_Location(void);
void Parse_CGNSSINFO(char *str);
void Trigger_Alarm(char *reason);
void Process_UART2_Data(void);
void Handle_UART_Line(char *line);
void SIM7680_Publish_Telemetry(void);
void Flush_UART2_Buffer(void);
uint8_t Send_AT_With_Retry(char *cmd, char *expected, uint8_t max_retries,
		uint16_t timeout_ms);
uint8_t Send_AT_Wait_Prompt_Retry(char *cmd, char *payload, uint8_t max_retries,
		uint16_t timeout_ms);
void Fall_Detection_Task(void);
void I2C_BusRecovery(void);
void Delay_With_Sensor_Service(uint32_t delay_ms);
void Poll_Panic_Button(void);
static uint8_t Phone_Is_Valid(const char *phone);
static MPU_Sample_Result_t MPU6050_DMA_Service(void);
static void UART2_RX_Service(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Định dạng float dạng fixed-point mà không kéo _printf_float vào Flash.
 * Hàm này giữ log UART/GPS dễ đọc nhưng chỉ dùng formatter số nguyên nhẹ. */
static char* Append_U32_Dec(char *dst, char *end, uint32_t value,
		uint8_t min_digits) {
	char reverse[10];
	uint8_t count = 0;

	do {
		reverse[count++] = (char) ('0' + (value % 10U));
		value /= 10U;
	} while (value != 0U && count < sizeof(reverse));

	while (count < min_digits && count < sizeof(reverse))
		reverse[count++] = '0';

	while (count > 0U && dst < end)
		*dst++ = reverse[--count];

	return dst;
}

static void Format_Fixed(char *out, size_t out_size, float value,
		uint8_t decimals) {
	char *dst;
	char *end;
	uint32_t scale = 1U;
	uint32_t whole;
	uint32_t fraction;

	if (out == NULL || out_size == 0U)
		return;

	dst = out;
	end = out + out_size - 1U;

	if (!isfinite(value)) {
		const char *special =
				isnan(value) ? "nan" : (value < 0.0f ? "-inf" : "inf");
		while (*special != '\0' && dst < end)
			*dst++ = *special++;
		*dst = '\0';
		return;
	}

	if (value < 0.0f) {
		if (dst < end)
			*dst++ = '-';
		value = -value;
	}

	for (uint8_t i = 0U; i < decimals; i++)
		scale *= 10U;

	whole = (uint32_t) value;
	fraction = (uint32_t) (((value - (float) whole) * (float) scale) + 0.5f);
	if (fraction >= scale) {
		whole++;
		fraction = 0U;
	}

	dst = Append_U32_Dec(dst, end, whole, 1U);
	if (decimals > 0U && dst < end) {
		*dst++ = '.';
		dst = Append_U32_Dec(dst, end, fraction, decimals);
	}
	*dst = '\0';
}

// =========================================================
// QUẢN LÝ BỘ NHỚ FLASH LƯU SỐ ĐIỆN THOẠI & GPS (FIX 3 & 6)
// =========================================================
#if ENABLE_RING_LOG
#define EVENT_LOG_SIZE 40
typedef struct {
	uint32_t seq;
	uint32_t t;
	uint8_t type;    // 0 = chuyển trạng thái, 1 = lỗi I2C
	uint8_t from_s, to_s;  // 7 = dữ liệu bẩn, 8 = AI từ chối, 9 = AI xác nhận
	float raw;
} EventLog_t;
EventLog_t event_log[EVENT_LOG_SIZE] = { 0 };
uint8_t event_log_idx = 0;
uint32_t event_log_seq = 0;

void Log_Event(uint8_t type, uint8_t from_s, uint8_t to_s, float raw) {
	event_log[event_log_idx].seq = ++event_log_seq;
	event_log[event_log_idx].t = HAL_GetTick();
	event_log[event_log_idx].type = type;
	event_log[event_log_idx].from_s = from_s;
	event_log[event_log_idx].to_s = to_s;
	event_log[event_log_idx].raw = raw;
	event_log_idx = (event_log_idx + 1) % EVENT_LOG_SIZE;
}
#else
/* Loại bỏ hoàn toàn lời gọi ghi log khỏi binary khi build bản final. */
#define Log_Event(type, from_s, to_s, raw) ((void) 0)
#endif

static uint8_t Phone_Is_Valid(const char *phone) {
	uint8_t digits = 0;
	uint8_t plus_seen = 0;

	if (phone == NULL)
		return 0;

	for (uint8_t i = 0; phone[i] != '\0'; i++) {
		if (i >= 19)
			return 0;
		if (phone[i] == '+') {
			if (i != 0 || plus_seen)
				return 0;
			plus_seen = 1;
		} else if (phone[i] >= '0' && phone[i] <= '9') {
			digits++;
		} else {
			return 0;
		}
	}
	return (digits >= 9 && digits <= 15);
}

uint8_t Save_Phone_To_Flash(const char *phone) {
	char buf[20] = { 0 };
	HAL_StatusTypeDef status = HAL_OK;

	if (!Phone_Is_Valid(phone))
		return 0;
	strncpy(buf, phone, 19);

	HAL_IWDG_Refresh(&hiwdg);
	if (HAL_FLASH_Unlock() != HAL_OK)
		return 0;

	FLASH_EraseInitTypeDef EraseInitStruct = { 0 };
	uint32_t PageError = 0xFFFFFFFFU;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_ADDR_PHONE;
	EraseInitStruct.NbPages = 1;
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);

	for (uint8_t i = 0; status == HAL_OK && i < sizeof(buf); i += 2) {
		uint16_t data = (uint16_t) (uint8_t) buf[i]
				| ((uint16_t) (uint8_t) buf[i + 1] << 8);
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
		FLASH_ADDR_PHONE + i, data);
	}
	HAL_FLASH_Lock();
	HAL_IWDG_Refresh(&hiwdg);

	if (status != HAL_OK)
		return 0;
	return (memcmp((const void*) FLASH_ADDR_PHONE, buf, sizeof(buf)) == 0);
}

void Load_Phone_From_Flash(void) {
	char *flash_data = (char*) FLASH_ADDR_PHONE;
	if (flash_data[0] != (char) 0xFF && flash_data[0] != 0x00
			&& memchr(flash_data, '\0', sizeof(phone_number)) != NULL
			&& Phone_Is_Valid(flash_data)) {
		strncpy(phone_number, flash_data, sizeof(phone_number) - 1);
		phone_number[sizeof(phone_number) - 1] = '\0';
	} else {
		strcpy(phone_number, "+84358751831");
	}
}

// =========================================================
// HÀM KHỞI TẠO VÀ ĐỌC MPU6050
// =========================================================
uint8_t MPU6050_ADDR_ACTIVE = 0x00;
static uint8_t mpu_dma_failed_cycles = 0;
static uint8_t mpu_low_g_candidate = 0;
static uint32_t mpu_dma_last_recovery_tick = 0;
static uint32_t mpu_dma_last_probe_tick = 0;

static uint8_t MPU6050_Write_Verify(uint8_t reg, uint8_t value, uint8_t mask) {
	uint8_t readback = 0;

	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR_ACTIVE, reg,
	I2C_MEMADD_SIZE_8BIT, &value, 1, 50) != HAL_OK) {
		mpu_last_hal_error = HAL_I2C_GetError(&hi2c1);
		return 0;
	}

	if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR_ACTIVE, reg,
	I2C_MEMADD_SIZE_8BIT, &readback, 1, 50) != HAL_OK) {
		mpu_last_hal_error = HAL_I2C_GetError(&hi2c1);
		return 0;
	}

	mpu_init_readback = readback;
	return ((readback & mask) == (value & mask));
}

static uint8_t MPU6050_Who_Is_Supported(uint8_t who) {
	/* Giữ đúng danh sách tương thích của code ban đầu. */
	return (who == 0x68 || who == 0x69 || who == 0x70 || who == 0x71
			|| who == 0x75 || who == 0x98);
}

void MPU6050_Init(void) {
	static const uint8_t addresses[] = { 0xD0, 0xD2 };
	uint8_t who = 0;
	HAL_StatusTypeDef status;

	mpu_ok = 0;
	mpu_sample_valid = 0;
	mpu_sensor_armed = 0;
	mpu_clean_sample_streak = 0;
	mpu_dma_busy = 0;
	mpu_dma_frame_ready = 0;
	mpu_dma_error_pending = 0;
	mpu_dma_error_code = HAL_I2C_ERROR_NONE;
	mpu_dma_failed_cycles = 0;
	mpu_low_g_candidate = 0;
	mpu_init_stage = 0;
	mpu_who_am_i = 0;
	mpu_init_readback = 0;
	MPU6050_ADDR_ACTIVE = 0;

	/* Thử cả hai địa chỉ AD0 và lưu WHO_AM_I để chẩn đoán module clone. */
	for (uint8_t i = 0; i < 2; i++) {
		who = 0;
		status = HAL_I2C_Mem_Read(&hi2c1, addresses[i], WHO_AM_I_REG,
		I2C_MEMADD_SIZE_8BIT, &who, 1, 50);
		if (status == HAL_OK) {
			mpu_who_am_i = who;
		}
		if (status == HAL_OK && MPU6050_Who_Is_Supported(who)) {
			MPU6050_ADDR_ACTIVE = addresses[i];
			break;
		} else if (status != HAL_OK) {
			mpu_last_hal_error = HAL_I2C_GetError(&hi2c1);
		}
	}

	if (MPU6050_ADDR_ACTIVE == 0) {
		mpu_init_stage = 1; /* Không probe được hoặc WHO_AM_I không hỗ trợ. */
		return;
	}
	mpu_init_stage = 2;

	/* Không ép DEVICE_RESET. Chọn clock PLL giống cấu hình đã test ổn định. */
	if (!MPU6050_Write_Verify(PWR_MGMT_1_REG, 0x01, 0x47)) {
		mpu_init_stage = 3;
		return;
	}
	HAL_Delay(50);

	/* DLPF=3 khớp cấu hình đã xác minh trên module WHO_AM_I=0x70. */
	if (!MPU6050_Write_Verify(CONFIG_REG, 0x03, 0x07)) {
		mpu_init_stage = 5;
		return;
	}

	if (!MPU6050_Write_Verify(SMPLRT_DIV_REG, 0x07, 0xFF)) {
		mpu_init_stage = 6;
		return;
	}
	if (!MPU6050_Write_Verify(ACCEL_CONFIG_REG, 0x18, 0x18)) {
		mpu_init_stage = 7;
		return;
	}
	if (!MPU6050_Write_Verify(GYRO_CONFIG_REG, 0x18, 0x18)) {
		mpu_init_stage = 8;
		return;
	}
	/* Chặn chạy nhầm với MSP chưa cấu hình I2C1_RX DMA. */
	if (hi2c1.hdmarx == NULL) {
		mpu_last_hal_error = 0xE106U;
		mpu_init_stage = 101;
		return;
	}

	mpu_last_hal_error = HAL_I2C_ERROR_NONE;
	mpu_init_stage = 100;
	mpu_ok = 1;
}

static uint8_t MPU6050_Frame_Is_Uniform(const uint8_t *data, uint8_t value) {
	for (uint8_t i = 0; i < MPU_DMA_FRAME_SIZE; i++) {
		if (data[i] != value)
			return 0;
	}
	return 1;
}

static void MPU6050_Record_Transport_Failure(uint32_t error_code) {
	uint32_t now = HAL_GetTick();

	mpu_sample_valid = 0;
	mpu_low_g_candidate = 0;
	mpu_last_hal_error = error_code;
	mpu_read_failure_count++;
	if (mpu_dma_failed_cycles < UINT8_MAX)
		mpu_dma_failed_cycles++;

	/* Chỉ recovery sau ba frame lỗi và tối đa một lần mỗi giây. */
	if (mpu_dma_failed_cycles >= 3U && now - mpu_dma_last_recovery_tick
	>= I2C_RECOVERY_MIN_INTERVAL_MS) {
		mpu_dma_last_recovery_tick = now;
		Log_Event(1, 99, 99, (float) error_code);
		I2C_BusRecovery();
		mpu_dma_failed_cycles = 0;
	}
}

static uint8_t MPU6050_DMA_Start(void) {
	HAL_StatusTypeDef status;

	if (!mpu_ok || mpu_dma_busy || mpu_dma_frame_ready)
		return 0;

	mpu_dma_error_pending = 0;
	mpu_dma_error_code = HAL_I2C_ERROR_NONE;
	mpu_dma_busy = 1;
	mpu_dma_start_tick = HAL_GetTick();
	status = HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR_ACTIVE,
	ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t*) mpu_dma_rx,
			MPU_DMA_FRAME_SIZE);
	if (status != HAL_OK) {
		mpu_dma_busy = 0;
		mpu_dma_error_code = HAL_I2C_GetError(&hi2c1);
		if (mpu_dma_error_code == HAL_I2C_ERROR_NONE)
			mpu_dma_error_code = 0xE100U; /* HAL_BUSY/HAL_ERROR khi start DMA. */
		mpu_dma_error_pending = 1;
		return 0;
	}
	return 1;
}

static MPU_Sample_Result_t MPU6050_Validate_DMA_Frame(void) {
	uint8_t rec_data[MPU_DMA_FRAME_SIZE];
	memcpy(rec_data, (const void*) mpu_dma_rx, sizeof(rec_data));

	mpu_sample_valid = 0;
	mpu_sample_saturated = 0;
	mpu_saturation_observed = 0;

	/* DMA hoàn tất vẫn có thể nhận payload hỏng do bus bị gián đoạn. */
	if (MPU6050_Frame_Is_Uniform(rec_data, 0x00)) {
		MPU6050_Record_Transport_Failure(0xE001U);
		return MPU_SAMPLE_INVALID;
	}
	if (MPU6050_Frame_Is_Uniform(rec_data, 0xFF)) {
		MPU6050_Record_Transport_Failure(0xE002U);
		return MPU_SAMPLE_INVALID;
	}

	int16_t ax_raw = (int16_t) (((uint16_t) rec_data[0] << 8) | rec_data[1]);
	int16_t ay_raw = (int16_t) (((uint16_t) rec_data[2] << 8) | rec_data[3]);
	int16_t az_raw = (int16_t) (((uint16_t) rec_data[4] << 8) | rec_data[5]);
	int16_t gx_raw = (int16_t) (((uint16_t) rec_data[8] << 8) | rec_data[9]);
	int16_t gy_raw = (int16_t) (((uint16_t) rec_data[10] << 8) | rec_data[11]);
	int16_t gz_raw = (int16_t) (((uint16_t) rec_data[12] << 8) | rec_data[13]);
	float ax = ax_raw / 2048.0f;
	float ay = ay_raw / 2048.0f;
	float az = az_raw / 2048.0f;
	float norm = sqrtf(ax * ax + ay * ay + az * az);
	uint8_t saturated = (ax_raw >= MPU_RAW_SATURATION_LIMIT
			|| ax_raw <= -MPU_RAW_SATURATION_LIMIT
			|| ay_raw >= MPU_RAW_SATURATION_LIMIT
			|| ay_raw <= -MPU_RAW_SATURATION_LIMIT
			|| az_raw >= MPU_RAW_SATURATION_LIMIT
			|| az_raw <= -MPU_RAW_SATURATION_LIMIT || !isfinite(norm)
			|| norm > 27.5f);

	if (saturated) {
		mpu_low_g_candidate = 0;
		mpu_sample_saturated = 1;
		mpu_saturation_observed = 1;
		mpu_rejected_saturation_count++;
		mpu_last_hal_error = 0xE004U;
		/* Bão hòa vật lý không phải lỗi transport nên không recovery bus. */
		mpu_dma_failed_cycles = 0;
		return MPU_SAMPLE_INVALID;
	}

	/* Một frame gần 0 g có thể là rơi thật hoặc byte cuối I2C bị sai.
	 * Giữ lại frame đầu, chỉ chấp nhận nếu frame kế tiếp cũng gần 0 g. */
	if (norm < MPU_LOW_G_CONFIRM_THRESHOLD) {
		if (mpu_low_g_candidate == 0U) {
			mpu_low_g_candidate = 1;
			mpu_low_g_candidate_count++;
			mpu_last_hal_error = 0xE005U;
			return MPU_SAMPLE_LOW_G_PENDING;
		}
		/* Đã có hai frame liên tiếp: chấp nhận các frame 0 g tiếp theo. */
		mpu_low_g_candidate = 2;
	} else {
		mpu_low_g_candidate = 0;
	}

	/* Chỉ công bố dữ liệu sau khi toàn bộ frame đã qua kiểm tra. */
	Accel_X_RAW = ax_raw;
	Accel_Y_RAW = ay_raw;
	Accel_Z_RAW = az_raw;
	Gyro_X_RAW = gx_raw;
	Gyro_Y_RAW = gy_raw;
	Gyro_Z_RAW = gz_raw;
	Ax = ax;
	Ay = ay;
	Az = az;
	Gx = gx_raw / 16.4f;
	Gy = gy_raw / 16.4f;
	Gz = gz_raw / 16.4f;

	mpu_dma_failed_cycles = 0;
	mpu_last_hal_error = HAL_I2C_ERROR_NONE;
	mpu_sample_valid = 1;
	return MPU_SAMPLE_VALID;
}

static MPU_Sample_Result_t MPU6050_DMA_Service(void) {
	uint32_t now = HAL_GetTick();
	MPU_Sample_Result_t result;

	/* MPU offline: thử khởi tạo lại tối đa một lần mỗi giây. */
	if (!mpu_ok) {
		if (now - mpu_dma_last_probe_tick >= 1000U) {
			mpu_dma_last_probe_tick = now;
			I2C_BusRecovery();
		}
		return MPU_SAMPLE_INVALID;
	}

	if (mpu_dma_busy) {
		if (now - mpu_dma_start_tick > MPU_DMA_TIMEOUT_MS) {
			/* Giữ cờ busy để recovery biết DMA cũ phải được abort. */
			mpu_dma_frame_ready = 0;
			mpu_dma_error_pending = 0;
			mpu_last_hal_error = HAL_I2C_ERROR_TIMEOUT;
			mpu_read_failure_count++;
			mpu_dma_last_recovery_tick = now;
			Log_Event(1, 99, 99, (float) HAL_I2C_ERROR_TIMEOUT);
			I2C_BusRecovery();
			return MPU_SAMPLE_INVALID;
		}
		/* DMA đáng ra hoàn tất trước chu kỳ TIM2 kế tiếp. */
		mpu_missed_deadline_count++;
		return MPU_SAMPLE_NONE;
	}

	if (mpu_dma_error_pending) {
		uint32_t error_code = mpu_dma_error_code;
		mpu_dma_error_pending = 0;
		if (error_code == HAL_I2C_ERROR_NONE)
			error_code = 0xE100U;
		MPU6050_Record_Transport_Failure(error_code);
		/* Không restart ngay trong cùng lượt xử lý lỗi: cho HAL/bus ổn định
		 * ít nhất một chu kỳ lấy mẫu trước lần thử mềm tiếp theo. */
		mpu_dma_retry_after_tick = HAL_GetTick() + MPU_DMA_SOFT_RETRY_MS;
		return MPU_SAMPLE_INVALID;
	}

	if (mpu_dma_frame_ready) {
		mpu_dma_frame_ready = 0;
		result = MPU6050_Validate_DMA_Frame();
		if (mpu_ok)
			(void) MPU6050_DMA_Start();
		return result;
	}

	/* Lần gọi đầu tiên hoặc lần soft-retry chỉ khởi động DMA khi đã hết
	 * khoảng nghỉ; frame sẽ được dùng ở tick kế tiếp. */
	if ((int32_t) (now - mpu_dma_retry_after_tick) >= 0)
		(void) MPU6050_DMA_Start();
	return MPU_SAMPLE_NONE;
}

// =========================================================
// KHỐI MÔ HÌNH AI (HYBRID AI VERIFIER)
// =========================================================
/* Mô hình subject-independent: train/validation/test tách hoàn toàn theo
 * SAxx/SExx; threshold được chọn trên validation với Recall mục tiêu >= 0.95. */
static const float SVM_BIAS = -2.114027035f;
static const float SVM_THRESHOLD = 0.298502437f;

static const float SVM_SCALE_MEAN[9] = { 1.470531493f, 0.715408304f,
		0.176148132f, 102.552663832f, 12.293068921f, 0.244471982f, 0.139911377f,
		0.193789518f, 140.788072442f };

static const float SVM_INV_SCALE_STD[9] = { 2.616941425f, 4.410884515f,
		7.166251416f, 0.037706623f, 0.099811535f, 5.685250640f, 7.299922443f,
		7.536215068f, 0.031956582f };

static const float SVM_COEF[9] = { 0.783307578f, 0.170590912f, -1.438782058f,
		2.780810686f, -0.743013734f, 0.914947443f, -0.087945354f, 0.467583992f,
		-1.951733812f };

typedef struct {
	uint16_t n;
	float mean;
	float m2;
} RunningStats_t;

static void RunningStats_Push(RunningStats_t *stats, float value) {
	stats->n++;
	float delta = value - stats->mean;
	stats->mean += delta / (float) stats->n;
	stats->m2 += delta * (value - stats->mean);
}

/* pandas.Series.std() trong pipeline train dùng ddof=1. */
static float RunningStats_SampleStd(const RunningStats_t *stats) {
	if (stats->n < 2)
		return 0.0f;
	float variance = stats->m2 / (float) (stats->n - 1U);
	return sqrtf(fmaxf(variance, 0.0f));
}

/* Khớp chính xác pipeline Python: atan2(sqrt(ay^2+az^2), ax+1e-6). */
static float AI_Calculate_Tilt(float ax, float ay, float az) {
	return atan2f(sqrtf(ay * ay + az * az), ax + 1.0e-6f) * RAD_TO_DEG_F;
}

static uint8_t AI_Predict_Fall(const float *features) {
	float score = SVM_BIAS;
	for (uint8_t i = 0; i < AI_FEATURE_COUNT; i++) {
		if (!isfinite(features[i])) {
			ai_last_score = -INFINITY;
			return 0;
		}
		float scaled_feature = (features[i] - SVM_SCALE_MEAN[i])
				* SVM_INV_SCALE_STD[i];
		score += scaled_feature * SVM_COEF[i];
	}
	ai_last_score = score;
	return (isfinite(score) && score > SVM_THRESHOLD) ? 1U : 0U;
}

// =========================================================
// REAL-TIME TASK: BỘ NÃO TÉ NGÃ (HYBRID ARCHITECTURE)
// =========================================================
void Fall_Detection_Task(void) {
	static float Ax_f = 0.0f, Ay_f = 0.0f, Az_f = 0.0f;
	static uint8_t filter_initialized = 0;
	static uint8_t freefall_counter = 0;
	static uint8_t impact_confirm_counter = 0;
	static uint8_t ai_invalid_sample_count = 0;
	static uint8_t saturation_after_freefall = 0;
	static uint8_t decimate_cnt = 0;
	static uint8_t valid_in_bucket = 0;
	static uint32_t last_valid_sample_tick = 0;
	static uint32_t last_impact_candidate_tick = 0;
	static float acc_sum_ax = 0, acc_sum_ay = 0, acc_sum_az = 0;
	static float acc_sum_gx = 0, acc_sum_gy = 0, acc_sum_gz = 0;

	/* Các biến context ở phạm vi static cục bộ, vì vậy gom reset tại đúng
	 * phạm vi này. Không xóa bộ lọc/motion state khi kết thúc một sự kiện. */
#define FALL_RESET_CONTEXT() do {                  \
		fall_state = 0U;                              \
		freefall_counter = 0U;                        \
		impact_confirm_counter = 0U;                  \
		ai_sample_count = 0U;                         \
		ai_invalid_sample_count = 0U;                 \
		saturation_after_freefall = 0U;               \
		decimate_cnt = 0U;                            \
		valid_in_bucket = 0U;                         \
		last_valid_sample_tick = 0U;                  \
		last_impact_candidate_tick = 0U;              \
		fall_state_enter_tick = 0U;                   \
		ai_collection_start_tick = 0U;                \
		acc_sum_ax = acc_sum_ay = acc_sum_az = 0.0f;  \
		acc_sum_gx = acc_sum_gy = acc_sum_gz = 0.0f;  \
	} while (0)

	uint32_t sample_now = HAL_GetTick();
	MPU_Sample_Result_t sample_result = MPU6050_DMA_Service();

	/* Timeout phải chạy ngay cả khi DMA chưa trả frame hoặc MPU offline. */
	if (fall_state
			== 1 && sample_now - fall_state_enter_tick > IMPACT_WAIT_TIMEOUT_MS) {
		FALL_RESET_CONTEXT();
		Log_Event(0, 1, 0, 0.0f);
	} else if (fall_state == 2 && sample_now - ai_collection_start_tick
	> AI_COLLECTION_TIMEOUT_MS) {
		FALL_RESET_CONTEXT();
		Log_Event(0, 2, 0, 0.0f);
	}

	/* DMA đang chạy hoặc frame 0 g đầu tiên đang chờ xác nhận: không coi là lỗi. */
	if (sample_result == MPU_SAMPLE_NONE
			|| sample_result == MPU_SAMPLE_LOW_G_PENDING)
		return;

	if (sample_result != MPU_SAMPLE_VALID) {
		mpu_clean_sample_streak = 0;
		mpu_sensor_armed = 0;
		impact_confirm_counter = 0;

		if (fall_state == 1 && mpu_sample_saturated)
			saturation_after_freefall = 1;

		if (fall_state == 2) {
			if (ai_invalid_sample_count < UINT8_MAX)
				ai_invalid_sample_count++;
			if (mpu_sample_saturated
					|| ai_invalid_sample_count > AI_MAX_INVALID_SAMPLES) {
				FALL_RESET_CONTEXT();
				Log_Event(0, 2, 7, mpu_sample_saturated ? 16.0f : 0.0f);
				return;
			}
		}

		if (fall_state == 0)
			freefall_counter = 0;
		return;
	}

	if (!mpu_sensor_armed) {
		if (mpu_clean_sample_streak < UINT16_MAX)
			mpu_clean_sample_streak++;
		if (mpu_clean_sample_streak >= SENSOR_CLEAN_ARM_SAMPLES)
			mpu_sensor_armed = 1;
	}

	if (fall_state == 0 && !mpu_sensor_armed) {
		freefall_counter = 0;
		impact_confirm_counter = 0;
		return;
	}

	// 1. Tách tín hiệu
	SVM_raw = sqrtf((Ax * Ax) + (Ay * Ay) + (Az * Az));
	if (!isfinite(SVM_raw) || SVM_raw > 28.0f) {
		mpu_sample_valid = 0;
		return;
	}

	if (!filter_initialized) {
		Ax_f = Ax;
		Ay_f = Ay;
		Az_f = Az;
		filter_initialized = 1;
	}
	Ax_f = 0.85f * Ax_f + 0.15f * Ax;
	Ay_f = 0.85f * Ay_f + 0.15f * Ay;
	Az_f = 0.85f * Az_f + 0.15f * Az;
	SVM_smooth = sqrtf((Ax_f * Ax_f) + (Ay_f * Ay_f) + (Az_f * Az_f));

	// 2. Nhận diện hành vi di chuyển (Dành cho Tracking GPS)
	if (SVM_smooth > 0.9f && SVM_smooth < 1.1f) {
		motion_active_count = 0;
		motion_stable_count++;
		if (motion_stable_count > 12000) {
			is_moving = 0;
			telemetry_interval = 1800000;
			motion_active_count = 0;
			motion_stable_count = 12001;
		}
	} else {
		motion_stable_count = 0;
		motion_active_count++;
		if (motion_active_count > 300) {
			is_moving = 1;
			telemetry_interval = 300000;
			motion_stable_count = 0;
			motion_active_count = 301;
		}
	}

	// 3. STATE MACHINE KẾT HỢP AI VERIFIER
	switch (fall_state) {
	case 0: // CHỜ RƠI TỰ DO (Lọc nhiễu sinh hoạt)
		/* Impact-first cần 2 mẫu sạch liên tiếp; spike đơn không mở AI. */
		if (SVM_raw > FALL_THRESHOLD) {
			if (impact_confirm_counter
					== 0 || sample_now - last_impact_candidate_tick
					<= IMPACT_CONFIRM_MAX_GAP_MS) {
				if (impact_confirm_counter < UINT8_MAX)
					impact_confirm_counter++;
			} else {
				impact_confirm_counter = 1;
			}
			last_impact_candidate_tick = sample_now;
			freefall_counter = 0;

			if (impact_confirm_counter < IMPACT_CONFIRM_SAMPLES)
				break;

			impact_confirm_counter = 0;
			fall_state = 2;
			Log_Event(0, 0, 2, SVM_raw);
			ai_sample_count = 0;
			ai_invalid_sample_count = 0;
			ai_collection_start_tick = sample_now;
			decimate_cnt = valid_in_bucket = 1;
			acc_sum_ax = Ax;
			acc_sum_ay = Ay;
			acc_sum_az = Az;
			acc_sum_gx = Gx;
			acc_sum_gy = Gy;
			acc_sum_gz = Gz;
			freefall_counter = 0;
			break;
		}
		impact_confirm_counter = 0;

		/* Các mẫu phải thực sự liên tiếp theo thời gian, không chỉ theo số lần gọi. */
		if (last_valid_sample_tick != 0
				&& sample_now - last_valid_sample_tick > 30)
			freefall_counter = 0;
		last_valid_sample_tick = sample_now;

		if (SVM_raw < FREE_FALL_THRESHOLD) {
			freefall_counter++;
			if (freefall_counter >= 5) {
				fall_state = 1;
				Log_Event(0, 0, 1, SVM_raw);
				fall_state_enter_tick = sample_now;
				freefall_counter = 0;
				saturation_after_freefall = 0;
			}
		} else {
			freefall_counter = 0;
		}
		break;

	case 1: // CHỜ VA CHẠM (IMPACT)
		impact_confirm_counter = 0;
		if (SVM_raw > FALL_THRESHOLD || saturation_after_freefall
				|| mpu_saturation_observed) {
			// CHUYỂN GIAO CHO AI: Bắt đầu gom mẫu
			fall_state = 2;
			Log_Event(0, 1, 2, SVM_raw);
			ai_sample_count = 0;
			ai_invalid_sample_count = 0;
			saturation_after_freefall = 0;
			/* Tính cả mẫu impact đầu tiên để khớp window bắt đầu tại peak. */
			decimate_cnt = 1;
			valid_in_bucket = 1;
			ai_collection_start_tick = sample_now;

			acc_sum_ax = Ax;
			acc_sum_ay = Ay;
			acc_sum_az = Az;
			acc_sum_gx = Gx;
			acc_sum_gy = Gy;
			acc_sum_gz = Gz;
		} else if (sample_now - fall_state_enter_tick > IMPACT_WAIT_TIMEOUT_MS) {
			Log_Event(0, 1, 0, SVM_raw);
			FALL_RESET_CONTEXT();
		}
		break;

	case 2: // AI VERIFIER: GOM 25 MẪU SAU VA CHẠM (TRUNG BÌNH 20 MẪU)
		if (sample_now - ai_collection_start_tick > AI_COLLECTION_TIMEOUT_MS) {
			Log_Event(0, 2, 0, SVM_raw);
			FALL_RESET_CONTEXT();
			break;
		}

		/* Không cho frame gần zero hoặc bão hòa đi vào SVM verifier. */
		if (SVM_raw < 0.10f || SVM_raw > 16.0f || mpu_sample_saturated) {
			if (ai_invalid_sample_count < UINT8_MAX)
				ai_invalid_sample_count++;
			if (mpu_sample_saturated
					|| ai_invalid_sample_count > AI_MAX_INVALID_SAMPLES) {
				Log_Event(0, 2, 7, SVM_raw);
				FALL_RESET_CONTEXT();
			}
			break;
		}

		decimate_cnt++;
		valid_in_bucket++;
		acc_sum_ax += Ax;
		acc_sum_ay += Ay;
		acc_sum_az += Az;
		acc_sum_gx += Gx;
		acc_sum_gy += Gy;
		acc_sum_gz += Gz;

		if (decimate_cnt >= AI_BUCKET_SAMPLES) {
			/* Lấy trung bình đúng số mẫu hợp lệ và chặn vượt mảng. */
			if (valid_in_bucket > 0 && ai_sample_count < AI_BATCH_SIZE) {
				ai_buffer[ai_sample_count].ax = acc_sum_ax / valid_in_bucket;
				ai_buffer[ai_sample_count].ay = acc_sum_ay / valid_in_bucket;
				ai_buffer[ai_sample_count].az = acc_sum_az / valid_in_bucket;
				ai_buffer[ai_sample_count].gx = acc_sum_gx / valid_in_bucket;
				ai_buffer[ai_sample_count].gy = acc_sum_gy / valid_in_bucket;
				ai_buffer[ai_sample_count].gz = acc_sum_gz / valid_in_bucket;
				ai_sample_count++;
			}

			// Reset bộ cộng dồn cho điểm dữ liệu tiếp theo
			acc_sum_ax = 0;
			acc_sum_ay = 0;
			acc_sum_az = 0;
			acc_sum_gx = 0;
			acc_sum_gy = 0;
			acc_sum_gz = 0;
			decimate_cnt = 0;
			valid_in_bucket = 0;

			// KHI ĐÃ GOM ĐỦ 5 GIÂY (25 MẪU) -> CHẠY AI
			if (ai_sample_count >= AI_BATCH_SIZE) {
				RunningStats_t acc_stats = { 0 }, gyro_stats = { 0 };
				RunningStats_t ax_stats = { 0 }, ay_stats = { 0 }, az_stats = {
						0 };
				float f_acc_max = 0.0f, f_acc_min = INFINITY;
				float f_gyro_max = 0.0f, sum_tilt = 0.0f;
				uint8_t window_valid = 1;

				for (uint8_t i = 0; i < AI_BATCH_SIZE; i++) {
					float ax = ai_buffer[i].ax;
					float ay = ai_buffer[i].ay;
					float az = ai_buffer[i].az;
					float gx = ai_buffer[i].gx;
					float gy = ai_buffer[i].gy;
					float gz = ai_buffer[i].gz;
					float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
					float gyro_norm = sqrtf(gx * gx + gy * gy + gz * gz);

					if (!isfinite(acc_norm) || !isfinite(gyro_norm)) {
						window_valid = 0;
						break;
					}
					RunningStats_Push(&acc_stats, acc_norm);
					RunningStats_Push(&gyro_stats, gyro_norm);
					RunningStats_Push(&ax_stats, ax);
					RunningStats_Push(&ay_stats, ay);
					RunningStats_Push(&az_stats, az);

					f_acc_max = fmaxf(f_acc_max, acc_norm);
					f_acc_min = fminf(f_acc_min, acc_norm);
					f_gyro_max = fmaxf(f_gyro_max, gyro_norm);
					sum_tilt += AI_Calculate_Tilt(ax, ay, az);
				}

				float features[AI_FEATURE_COUNT] = { f_acc_max, f_acc_min,
						RunningStats_SampleStd(&acc_stats), f_gyro_max,
						RunningStats_SampleStd(&gyro_stats),
						RunningStats_SampleStd(&ax_stats),
						RunningStats_SampleStd(&ay_stats),
						RunningStats_SampleStd(&az_stats), sum_tilt
								/ (float) AI_BATCH_SIZE };

				uint8_t ai_result =
						window_valid ? AI_Predict_Fall(features) : 0U;
				char ai_result_msg[112];
				char score_text[20];
				char threshold_text[20];
				if (!window_valid)
					ai_last_score = -INFINITY;
				Format_Fixed(score_text, sizeof(score_text), ai_last_score, 3U);
				Format_Fixed(threshold_text, sizeof(threshold_text),
						SVM_THRESHOLD, 3U);

				if (ai_result) {
					fall_detected_flag = 1; // HÚ CÒI!
					Log_Event(0, 2, 9, SVM_raw);
					snprintf(ai_result_msg, sizeof(ai_result_msg),
							"\r\n[AI] FALL score=%s threshold=%s invalid=%u\r\n",
							score_text, threshold_text,
							(unsigned int) ai_invalid_sample_count);
				} else {
					Log_Event(0, 2, 8, SVM_raw);
					snprintf(ai_result_msg, sizeof(ai_result_msg),
							"\r\n[AI] REJECT score=%s threshold=%s invalid=%u\r\n",
							score_text, threshold_text,
							(unsigned int) ai_invalid_sample_count);
				}
				HAL_UART_Transmit(&huart1, (uint8_t*) ai_result_msg,
						strlen(ai_result_msg), 100);

				FALL_RESET_CONTEXT();

			}
		}
		break;

	default:
		/* Tự phục hồi nếu RAM bị nhiễu làm fall_state ra ngoài 0..2. */
		Log_Event(0, fall_state, 0, SVM_raw);
		FALL_RESET_CONTEXT();
		break;
	}

#undef FALL_RESET_CONTEXT
}

// =========================================================
// LÕI GIAO TIẾP AT COMMAND
// =========================================================
void Poll_Panic_Button(void) {
	static GPIO_PinState raw_previous = GPIO_PIN_SET;
	static GPIO_PinState stable_state = GPIO_PIN_SET;
	static uint32_t change_tick = 0;
	GPIO_PinState raw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	uint32_t now = HAL_GetTick();

	if (raw != raw_previous) {
		raw_previous = raw;
		change_tick = now;
	}

	if (raw != stable_state && now - change_tick >= 50) {
		stable_state = raw;
		if (stable_state == GPIO_PIN_RESET)
			panic_button_flag = 1;
	}
}

void Delay_With_Sensor_Service(uint32_t delay_ms) {
	uint32_t start = HAL_GetTick();

	while (HAL_GetTick() - start < delay_ms) {
		Poll_Panic_Button();
		if (mpu_read_flag) {
			mpu_read_flag = 0;
			Fall_Detection_Task();
		}
#if ENABLE_SIM
		/* Xả URC/MQTT trong delay, nhưng không tranh byte phản hồi với hàm AT. */
		if (!sim_at_transaction_active) {
			UART2_RX_Service();
			Process_UART2_Data();
		}
#endif
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(1);
	}
}

void Flush_UART2_Buffer(void) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	rx3_tail = rx3_head;
	rx3_desync = 0;
	if (primask == 0U)
		__enable_irq();

	/* Không để nửa dòng cũ ghép với phản hồi/URC sau lần flush. */
	process_idx = 0U;
	process_drop_line = 0U;
	expecting_phone = 0U;
	process_buf[0] = '\0';
}

/* Chỉ main gọi hàm này. ISR chỉ đặt uart2_restart_pending để tránh chạy
 * HAL_UART_AbortReceive() trong ngắt và tránh USART2 bị điếc do HAL_BUSY. */
static void UART2_RX_Service(void) {
#if ENABLE_SIM
	uint32_t primask;

	if (sim_at_transaction_active)
		return;

	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
		uart2_restart_pending = 1U;

	if (!uart2_restart_pending)
		return;

	primask = __get_PRIMASK();
	__disable_irq();
	uart2_restart_pending = 0U;
	if (primask == 0U)
		__enable_irq();

	(void) HAL_UART_AbortReceive(&huart2);
	__HAL_UART_CLEAR_OREFLAG(&huart2);
	__HAL_UART_CLEAR_NEFLAG(&huart2);
	__HAL_UART_CLEAR_FEFLAG(&huart2);
	__HAL_UART_CLEAR_PEFLAG(&huart2);
	Flush_UART2_Buffer();

	if (HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx3_byte, 1U) != HAL_OK)
		uart2_restart_pending = 1U;
#endif
}

static void SIM_Abort_Current_Transaction(void) {
	uint8_t esc = 0x1B;
	(void) HAL_UART_Transmit(&huart2, &esc, 1, 100);
	/* Không được xóa trạng thái MQTT tại đây. Hàm này còn được dùng cho
	 * GPS/SMS/AT chung; timeout GPS không có nghĩa kết nối MQTT đã mất.
	 * Chỉ MQTT URC hoặc chính hàm MQTT thất bại mới được đặt cờ về 0. */
	Flush_UART2_Buffer();
	sim_at_transaction_active = 0;
}

uint8_t Send_AT_With_Retry(char *cmd, char *expected, uint8_t max_retries,
		uint16_t timeout_ms) {
	char line[256];
	uint16_t idx = 0;

	for (uint8_t retry = 1; retry <= max_retries; retry++) {
#if ENABLE_AT_TRACE
		char debug_msg[64];
		snprintf(debug_msg, sizeof(debug_msg), "\r\n[GUI %d/%d] %s", retry,
				max_retries, cmd);
		HAL_UART_Transmit(&huart1, (uint8_t*) debug_msg, strlen(debug_msg),
				100);
#endif

		/* Xử lý URC/MQTT đã nhận trước khi bắt đầu transaction mới. */
		Process_UART2_Data();
		UART2_RX_Service();
		sim_at_transaction_active = 1;
		Flush_UART2_Buffer();

		HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), 1000);

		uint32_t start = HAL_GetTick();
		uint8_t status = 0;
		idx = 0;
		memset(line, 0, sizeof(line));

		while (HAL_GetTick() - start < timeout_ms) {
			Poll_Panic_Button();
			// FIX 2: CHỐNG MÙ - Vẫn quét té ngã khi đang chờ Mạng
			if (mpu_read_flag == 1) {
				mpu_read_flag = 0;
				Fall_Detection_Task();
			}

			// Nếu phát hiện té ngã hoặc bấm nút, lập tức HỦY BỎ việc chờ mạng (return 0)
			if (!alarm_transmit_active
					&& (fall_detected_flag == 1 || panic_button_flag == 1)) {
				HAL_UART_Transmit(&huart1,
						(uint8_t*) "\r\n[SYS] NGAT MANG - UU TIEN CAP CUU!\r\n",
						38, 100);
				SIM_Abort_Current_Transaction();
				return 0;
			}
			if (rx3_desync) {
				SIM_Abort_Current_Transaction();
				return 0;
			}

			if (rx3_tail != rx3_head) {
				char c = rx3_buf[rx3_tail];
				rx3_tail = (rx3_tail + 1) % RX3_BUF_SIZE;
#if ENABLE_AT_TRACE
				HAL_UART_Transmit(&huart1, (uint8_t*) &c, 1, 10);
#endif

				if (c == '\n') {
					line[idx] = '\0';
					Handle_UART_Line(line);

					if (expected != NULL && strstr(line, expected) != NULL) {
						sim_at_transaction_active = 0;
						return 1;
					}
					if (strstr(line, "ERROR")) {
						status = 2;
						break;
					}

					idx = 0;
					memset(line, 0, sizeof(line));
				} else if (c != '\r') {
					if (idx < sizeof(line) - 1U)
						line[idx++] = c;
					else {
						status = 3;
						uart_line_overflow_count++;
						break;
					}
				}
			}
			HAL_IWDG_Refresh(&hiwdg);
		}

		if (status == 2)
			HAL_UART_Transmit(&huart1,
					(uint8_t*) "\r\n=> [LOI] SIM bao ERROR.\r\n", 27, 100);
		else if (status == 3)
			HAL_UART_Transmit(&huart1,
					(uint8_t*) "\r\n=> [LOI] Dong SIM qua dai.\r\n", 30, 100);
		else
			HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n=> [LOI] Timeout.\r\n",
					21, 100);
		SIM_Abort_Current_Transaction();
		Delay_With_Sensor_Service(1500);
	}
	sim_at_transaction_active = 0;
	return 0;
}

static uint8_t SIM_Wait_For_Token(const char *expected, uint16_t timeout_ms) {
	char line[256] = { 0 };
	uint16_t idx = 0;
	uint32_t start = HAL_GetTick();
	sim_at_transaction_active = 1;

	while (HAL_GetTick() - start < timeout_ms) {
		Poll_Panic_Button();
		if (mpu_read_flag) {
			mpu_read_flag = 0;
			Fall_Detection_Task();
		}
		if ((!alarm_transmit_active && (fall_detected_flag || panic_button_flag))
				|| rx3_desync) {
			SIM_Abort_Current_Transaction();
			return 0;
		}

		if (rx3_tail != rx3_head) {
			char c = rx3_buf[rx3_tail];
			rx3_tail = (rx3_tail + 1U) % RX3_BUF_SIZE;
			if (c == '\n') {
				line[idx] = '\0';
				Handle_UART_Line(line);
				if (expected != NULL && strstr(line, expected) != NULL) {
					if (strcmp(expected, "+CMQTTSUB:") == 0) {
						uint8_t result =
								(strstr(line, "0,0") != NULL) ? 1U : 0U;
						sim_at_transaction_active = 0;
						return result;
					}
					sim_at_transaction_active = 0;
					return 1;
				}
				if (strstr(line, "ERROR") != NULL) {
					sim_at_transaction_active = 0;
					return 0;
				}
				idx = 0;
			} else if (c != '\r') {
				if (idx < sizeof(line) - 1U)
					line[idx++] = c;
				else {
					uart_line_overflow_count++;
					sim_at_transaction_active = 0;
					return 0;
				}
			}
		}
		HAL_IWDG_Refresh(&hiwdg);
	}
	sim_at_transaction_active = 0;
	return 0;
}

uint8_t Send_AT_Wait_Prompt_Retry(char *cmd, char *payload, uint8_t max_retries,
		uint16_t timeout_ms) {
	for (uint8_t retry = 1; retry <= max_retries; retry++) {
#if ENABLE_AT_TRACE
		char debug_msg[64];
		snprintf(debug_msg, sizeof(debug_msg), "\r\n[PROMPT %d/%d] %s", retry,
				max_retries, cmd);
		HAL_UART_Transmit(&huart1, (uint8_t*) debug_msg, strlen(debug_msg),
				100);
#endif

		Process_UART2_Data();
		UART2_RX_Service();
		sim_at_transaction_active = 1;
		Flush_UART2_Buffer();
		HAL_UART_Transmit(&huart2, (uint8_t*) cmd, strlen(cmd), 1000);

		uint32_t start = HAL_GetTick();
		uint8_t prompt_found = 0;

		while (HAL_GetTick() - start < timeout_ms) {
			Poll_Panic_Button();
			// CHỐNG MÙ FALL DETECTION
			if (mpu_read_flag == 1) {
				mpu_read_flag = 0;
				Fall_Detection_Task();
			}

			// Nếu phát hiện té ngã hoặc bấm nút, lập tức HỦY BỎ việc chờ mạng (return 0)
			if (!alarm_transmit_active
					&& (fall_detected_flag == 1 || panic_button_flag == 1)) {
				HAL_UART_Transmit(&huart1,
						(uint8_t*) "\r\n[SYS] NGAT MANG - UU TIEN CAP CUU!\r\n",
						38, 100);
				SIM_Abort_Current_Transaction();
				return 0;
			}
			if (rx3_desync) {
				SIM_Abort_Current_Transaction();
				return 0;
			}

			if (rx3_tail != rx3_head) {
				char c = rx3_buf[rx3_tail];
				rx3_tail = (rx3_tail + 1) % RX3_BUF_SIZE;
#if ENABLE_AT_TRACE
				HAL_UART_Transmit(&huart1, (uint8_t*) &c, 1, 10);
#endif
				if (c == '>') {
					prompt_found = 1;
					break;
				}
			}
			HAL_IWDG_Refresh(&hiwdg);
		}

		if (prompt_found) {
			Delay_With_Sensor_Service(50);
#if ENABLE_AT_TRACE
			HAL_UART_Transmit(&huart1, (uint8_t*) "[PAYLOAD] ", 10, 100);
			HAL_UART_Transmit(&huart1, (uint8_t*) payload, strlen(payload),
					1000);
			HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, 100);
#endif
			HAL_UART_Transmit(&huart2, (uint8_t*) payload, strlen(payload),
					1000);
			/* ACK/terminator phụ thuộc lệnh: MQTT chờ OK, SMS còn cần Ctrl-Z. */
			Delay_With_Sensor_Service(50);
			sim_at_transaction_active = 0;
			return 1;
		} else {
			HAL_UART_Transmit(&huart1,
					(uint8_t*) "\r\n=> [LOI] Khong thay '>'\r\n", 27, 100);
			SIM_Abort_Current_Transaction();
			Delay_With_Sensor_Service(1000);
		}
	}
	sim_at_transaction_active = 0;
	return 0;
}

// =========================================================
// XỬ LÝ DỮ LIỆU GPS & MẠNG
// =========================================================
void Parse_CGNSSINFO(char *str) {
	char *p = strstr(str, "+CGNSSINFO:");
	if (!p)
		return;
	p += 11;
	int comma_count = 0, lat_idx = 0, lon_idx = 0;
	char lat_str[20] = { 0 }, lon_str[20] = { 0 }, ns = 'N', ew = 'E';

	while (*p && *p != '\r' && *p != '\n') {
		if (*p == ',')
			comma_count++;
		else if (*p != ' ') {
			if (comma_count == 5 && lat_idx < 19)
				lat_str[lat_idx++] = *p;
			if (comma_count == 6)
				ns = *p;
			if (comma_count == 7 && lon_idx < 19)
				lon_str[lon_idx++] = *p;
			if (comma_count == 8)
				ew = *p;
		}
		p++;
	}

	if (strlen(lat_str) > 0 && strlen(lon_str) > 0) {
		float raw_lat = atof(lat_str);
		float raw_lon = atof(lon_str);
		if (raw_lat != 0 && raw_lon != 0) {
			gps_valid = 1;
			latitude = (ns == 'S') ? -raw_lat : raw_lat;
			longitude = (ew == 'W') ? -raw_lon : raw_lon;
		} else
			gps_valid = 0;
	} else
		gps_valid = 0;
}

void Handle_UART_Line(char *line) {
	if (strncmp(line, "+CGNSSINFO:", 11) == 0) {

#if ENABLE_AT_TRACE
		/* Chỉ in GPS raw khi trace; chuỗi dài này làm trễ tick 100 Hz. */
		HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n[RAW GPS] ", 12, 100);
		HAL_UART_Transmit(&huart1, (uint8_t*) line, strlen(line), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n", 2, 100);
#endif

		Parse_CGNSSINFO(line);

	} else if (strstr(line, "+CMQTTDISC") != NULL
			|| strstr(line, "+CMQTTCONNLOST") != NULL) {
		mqtt_connected = 0;
		expecting_phone = 0;

	} else if (strncmp(line, "+CMQTTRXPAYLOAD:", 16) == 0) {
		/* Payload cấu hình chỉ chấp nhận độ dài của một số E.164. */
		char *last_comma = strrchr(line, ',');
		long payload_len =
				(last_comma != NULL) ? strtol(last_comma + 1, NULL, 10) : 0;
		expecting_phone = (payload_len >= 9 && payload_len <= 16) ? 1U : 0U;

	} else if (expecting_phone) {
		// KIỂM TRA LỆNH LƯU SỐ ĐIỆN THOẠI
		char new_phone[20] = { 0 };
		int i = 0, j = 0;
		// Quét trực tiếp trên biến 'line'
		while (line[j] && i < 16) {
			if ((line[j] >= '0' && line[j] <= '9') || line[j] == '+')
				new_phone[i++] = line[j];
			j++;
		}

		if (Phone_Is_Valid(new_phone)) {
			strcpy(pending_phone_number, new_phone);
			new_phone_pending_flag = 1;
			HAL_UART_Transmit(&huart1,
					(uint8_t*) "\r\n[SYS] DA NHAN SDT MOI, CHO GHI FLASH...\r\n",
					43, 100);
		}
		expecting_phone = 0;
	}
}
void Process_UART2_Data(void) {
	uint16_t budget = UART_PROCESS_BUDGET;

	if (rx3_desync) {
		/* Khi ring overflow, tuyệt đối không ghép phần đuôi với dòng cũ. */
		Flush_UART2_Buffer();
		process_idx = 0;
		process_drop_line = 0;
		expecting_phone = 0;
		return;
	}

	while (rx3_tail != rx3_head && budget-- > 0U) {
		if (rx3_desync) {
			Flush_UART2_Buffer();
			process_idx = 0;
			process_drop_line = 0;
			expecting_phone = 0;
			return;
		}
		char c = rx3_buf[rx3_tail];
		rx3_tail = (rx3_tail + 1) % RX3_BUF_SIZE;

		if (c == '\n' || c == '\r') {
			if (!process_drop_line && process_idx > 0) {
				process_buf[process_idx] = '\0';
				Handle_UART_Line(process_buf);
			}
			process_idx = 0;
			process_drop_line = 0;
		} else {
			if (!process_drop_line && process_idx < sizeof(process_buf) - 1U)
				process_buf[process_idx++] = c;
			else if (!process_drop_line) {
				process_idx = 0;
				process_drop_line = 1;
				expecting_phone = 0;
				uart_line_overflow_count++;
			}
		}
		HAL_IWDG_Refresh(&hiwdg);
	}
}

// =========================================================
// KHỞI TẠO CÁC CHỨC NĂNG SIM VỚI RETRY
// =========================================================
void SIM7680_Network_Init(void) {
	HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n[SYS] === KHOI TAO MANG ===\r\n",
			31, 100);

	// Thử kết nối AT tối đa 5 lần thay vì vô hạn
	for (int i = 0; i < 5; i++) {
		if (!alarm_transmit_active && (fall_detected_flag || panic_button_flag))
			return; // Ưu tiên cứu nạn ngay lập tức
		if (Send_AT_With_Retry("AT\r\n", "OK", 1, 2000))
			break;
		HAL_UART_Transmit(&huart1, (uint8_t*) "[SYS] Cho SIM len tieng...\r\n",
				28, 100);
		Delay_With_Sensor_Service(1000);
		HAL_IWDG_Refresh(&hiwdg);
	}

	/* Tắt echo để giảm một nửa lưu lượng RX và tránh parser nhận lại command. */
	Send_AT_With_Retry("ATE0\r\n", "OK", 3, 2000);
	Send_AT_With_Retry("AT+CMGF=1\r\n", "OK", 3, 2000);

	// Tìm sóng 4G tối đa 10 lần (khoảng 30 giây) thay vì vô hạn
	for (int i = 0; i < 10; i++) {
		if (!alarm_transmit_active && (fall_detected_flag || panic_button_flag))
			return; // Ưu tiên cứu nạn

		if (Send_AT_With_Retry("AT+CGREG?\r\n", "+CGREG: 0,1", 1, 2000)
				|| Send_AT_With_Retry("AT+CGREG?\r\n", "+CGREG: 0,5", 1,
						2000)) {
			HAL_UART_Transmit(&huart1,
					(uint8_t*) "\r\n[SYS] DA CO SONG 4G!\r\n", 24, 100);
			return; // Thoát thành công
		}
		HAL_UART_Transmit(&huart1,
				(uint8_t*) "[SYS] Dang do tim song 4G...\r\n", 30, 100);
		Delay_With_Sensor_Service(2000);
		HAL_IWDG_Refresh(&hiwdg);
	}
	HAL_UART_Transmit(&huart1,
			(uint8_t*) "\r\n[SYS] KHONG TIM THAY MANG, CHAY OFFLINE!\r\n", 44,
			100);
}

void SIM7680_GNSS_Init(void) {
	HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n[SYS] === KHOI TAO GPS ===\r\n",
			30, 100);

	if (!Send_AT_With_Retry("AT+CGNSSPWR=1\r\n", "OK", 2, 2000)) {
		HAL_UART_Transmit(&huart1,
				(uint8_t*) "[SYS] Loi bat nguon GPS (Bo qua)\r\n", 34, 100);
	}
	if (!Send_AT_With_Retry("AT+CGNSSMODE=3\r\n", "OK", 2, 2000)) {
		HAL_UART_Transmit(&huart1,
				(uint8_t*) "[SYS] Loi set Mode GPS (Bo qua)\r\n", 33, 100);
	}
}

void SIM7680_MQTT_Init(void) {
	char accq_cmd[96];
	char client_id[32];
	snprintf(client_id, sizeof(client_id), "FD_%08lX%08lX",
			(unsigned long) HAL_GetUIDw1(), (unsigned long) HAL_GetUIDw0());
	snprintf(accq_cmd, sizeof(accq_cmd), "AT+CMQTTACCQ=0,\"%s\"\r\n",
			client_id);

	mqtt_connected = 0;
	HAL_UART_Transmit(&huart1,
			(uint8_t*) "\r\n[SYS] === BAT DAU KHOI TAO MQTT ===\r\n", 39, 100);
	Send_AT_With_Retry("AT+CMQTTDISC=0,60\r\n", "OK", 2, 2000);
	Send_AT_With_Retry("AT+CMQTTREL=0\r\n", "OK", 2, 2000);
	Send_AT_With_Retry("AT+CMQTTSTOP\r\n", "OK", 2, 2000);

	if (!Send_AT_With_Retry("AT+CMQTTSTART\r\n", "OK", 3, 3000))
		return;
	if (!Send_AT_With_Retry(accq_cmd, "OK", 3, 3000))
		return;

	if (!Send_AT_With_Retry(
			"AT+CMQTTCONNECT=0,\"tcp://broker.emqx.io:1883\",60,1\r\n", "0,0",
			3, 8000)) {
		HAL_UART_Transmit(&huart1,
				(uint8_t*) "\r\n[MQTT] KET NOI THAT BAI (Bo qua Subscribe)\r\n",
				46, 100);
		return;
	}

	char sub_cmd[50];
	char topic_sub[] = "thiet_bi/cai_dat";
	snprintf(sub_cmd, sizeof(sub_cmd), "AT+CMQTTSUB=0,%u,1\r\n",
			(unsigned int) strlen(topic_sub));

	if (Send_AT_Wait_Prompt_Retry(sub_cmd, topic_sub, 3, 5000)
			&& SIM_Wait_For_Token("OK", 3000)
			&& SIM_Wait_For_Token("+CMQTTSUB:", 5000)) {
		mqtt_connected = 1;
		HAL_UART_Transmit(&huart1,
				(uint8_t*) "\r\n[MQTT] KHOI TAO HOAN TAT!\r\n", 29, 100);
	}
}

void Update_GPS_Location(void) {
	/* Mỗi lần hỏi GPS phải bắt đầu bằng trạng thái chưa có fix. Nếu lệnh lỗi
	 * hoặc GNSS chưa khóa vệ tinh, tuyệt đối không tái sử dụng tọa độ cũ và
	 * gắn nhãn LIVE cho lần cảnh báo hiện tại. */
	gps_valid = 0;
	latitude = 0.0f;
	longitude = 0.0f;
	(void) Send_AT_With_Retry("AT+CGNSSINFO\r\n", "OK", 1, 2000);
}

void Send_Emergency_SMS(char *reason) {
	char message[256], cmd_buffer[100];
	char lat_text[24], lon_text[24];
	uint8_t ctrl_z = 0x1A;
	const char *sms_phone = phone_number;

	/* Nếu App vừa đổi số trong phiên MQTT này, dùng ngay số mới cho cảnh báo
	 * hiện tại; việc ghi Flash vẫn được main loop rate-limit sau đó. */
	if (new_phone_pending_flag && Phone_Is_Valid(pending_phone_number))
		sms_phone = pending_phone_number;

	if (gps_valid) {
		Format_Fixed(lat_text, sizeof(lat_text), latitude, 6U);
		Format_Fixed(lon_text, sizeof(lon_text), longitude, 6U);
		snprintf(message, sizeof(message),
				"CANH BAO: %s. Pin: %d%%. Vi tri: https://maps.google.com/maps?q=%s,%s",
				reason, battery_percent, lat_text, lon_text);
	} else {
		snprintf(message, sizeof(message),
				"CANH BAO: %s. Pin: %d%%. Chua co tin hieu GPS.", reason,
				battery_percent);
	}
	HAL_UART_Transmit(&huart1,
			(uint8_t*) "\r\n[SIM] === BAT DAU GUI SMS ===\r\n", 33, 100);
	Send_AT_With_Retry("AT+CMGF=1\r\n", "OK", 3, 2000);

	char clean_phone[20] = { 0 };
	size_t idx = 0U;
	size_t phone_len = 0U;
	while (phone_len < sizeof(phone_number) && sms_phone[phone_len] != '\0')
		phone_len++;
	for (size_t i = 0U; i < phone_len; i++) {
		char c = sms_phone[i];
		if ((c >= '0' && c <= '9') || c == '+') {
			if (idx >= sizeof(clean_phone) - 1U)
				break;
			clean_phone[idx++] = c;
		}
	}
	clean_phone[idx] = '\0';
	snprintf(cmd_buffer, sizeof(cmd_buffer), "AT+CMGS=\"%s\"\r\n", clean_phone);

	if (Send_AT_Wait_Prompt_Retry(cmd_buffer, message, 3, 5000)) {
		Delay_With_Sensor_Service(200);
		HAL_UART_Transmit(&huart2, &ctrl_z, 1, 1000);
		if (SIM_Wait_For_Token("+CMGS:", 10000)
				&& SIM_Wait_For_Token("OK", 3000)) {
			HAL_UART_Transmit(&huart1,
					(uint8_t*) "\r\n[SIM] DA GUI XONG SMS!\r\n", 26, 100);
		} else {
			SIM_Abort_Current_Transaction();
			HAL_UART_Transmit(&huart1,
					(uint8_t*) "\r\n[SIM] GUI SMS THAT BAI!\r\n", 27, 100);
		}
	} else {
		uint8_t esc_char = 0x1B; // Ký tự ESCAPE để thoát khỏi chế độ SMS an toàn
		HAL_UART_Transmit(&huart2, &esc_char, 1, 1000);
		Delay_With_Sensor_Service(300);
	}
}

void SIM7680_Publish_To_App(char *reason) {
	if (!mqtt_connected)
		return;
	char payload[250], at_cmd[50];
	char lat_text[24], lon_text[24];

	if (gps_valid) {
		Format_Fixed(lat_text, sizeof(lat_text), latitude, 6U);
		Format_Fixed(lon_text, sizeof(lon_text), longitude, 6U);
		snprintf(payload, sizeof(payload),
				"{\"alert\":\"%s\", \"lat\":%s, \"lon\":%s, \"bat\":%d, \"gps\":\"LIVE\"}",
				reason, lat_text, lon_text, battery_percent);
	} else {
		snprintf(payload, sizeof(payload),
				"{\"alert\":\"%s\", \"lat\":0.0, \"lon\":0.0, \"bat\":%d, \"gps\":\"NONE\"}",
				reason, battery_percent);
	}

	snprintf(at_cmd, sizeof(at_cmd), "AT+CMQTTTOPIC=0,%u\r\n",
			(unsigned int) strlen("thiet_bi/bao_dong"));
	if (!Send_AT_Wait_Prompt_Retry(at_cmd, "thiet_bi/bao_dong", 3, 5000)
			|| !SIM_Wait_For_Token("OK", 3000)) {
		mqtt_connected = 0; // BẮT BỆNH ZOMBIE
		return;
	}

	snprintf(at_cmd, sizeof(at_cmd), "AT+CMQTTPAYLOAD=0,%u\r\n",
			(unsigned int) strlen(payload));
	if (!Send_AT_Wait_Prompt_Retry(at_cmd, payload, 3, 5000)
			|| !SIM_Wait_For_Token("OK", 3000)) {
		mqtt_connected = 0; // BẮT BỆNH ZOMBIE
		return;
	}

	if (!Send_AT_With_Retry("AT+CMQTTPUB=0,1,60\r\n", "OK", 3, 5000)) {
		mqtt_connected = 0; // BẮT BỆNH ZOMBIE
	}
}

uint32_t last_alarm_tick = 0;
void Trigger_Alarm(char *reason) {
	uint32_t now = HAL_GetTick();
	if (!first_alarm && (now - last_alarm_tick < ALARM_COOLDOWN_MS))
		return;
	first_alarm = 0;
	last_alarm_tick = now;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);

#if ENABLE_SIM
	/* Chốt GPS và pin ngay lúc cảnh báo để payload không dùng số liệu cũ. */
	alarm_transmit_active = 1U;
	Update_GPS_Location();
	battery_percent = Read_Battery_Percent();

	SIM7680_MQTT_Init();
	/* Nhận retained config trước SMS để số vừa đổi trên App có hiệu lực ngay. */
	Delay_With_Sensor_Service(SIM_CONFIG_RX_WINDOW_MS);
	Send_Emergency_SMS(reason);
	SIM7680_Publish_To_App(reason);
	alarm_transmit_active = 0U;
#else
	(void) reason;
	HAL_UART_Transmit(&huart1,
			(uint8_t*) "\r\n[ALARM] SIM OFF - CANH BAO TAI CHO\r\n",
			(uint16_t) (sizeof("\r\n[ALARM] SIM OFF - CANH BAO TAI CHO\r\n") - 1U),
			100);
#endif
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
}

void SIM7680_Publish_Telemetry(void) {
	if (!mqtt_connected)
		return;
	char payload[180], at_cmd[50];
	char status_str[10];
	char lat_text[24], lon_text[24];

	strcpy(status_str, is_moving ? "MOVING" : "STILL");

	if (gps_valid) {
		Format_Fixed(lat_text, sizeof(lat_text), latitude, 6U);
		Format_Fixed(lon_text, sizeof(lon_text), longitude, 6U);
		snprintf(payload, sizeof(payload),
				"{\"lat\":%s, \"lon\":%s, \"bat\":%d, \"status\":\"%s\", \"gps\":\"LIVE\"}",
				lat_text, lon_text, battery_percent, status_str);
	} else {
		snprintf(payload, sizeof(payload),
				"{\"lat\":0.0, \"lon\":0.0, \"bat\":%d, \"status\":\"%s\", \"gps\":\"NONE\"}",
				battery_percent, status_str);
	}

	snprintf(at_cmd, sizeof(at_cmd), "AT+CMQTTTOPIC=0,%u\r\n",
			(unsigned int) strlen("thiet_bi/telemetry"));
	if (!Send_AT_Wait_Prompt_Retry(at_cmd, "thiet_bi/telemetry", 2, 2000)
			|| !SIM_Wait_For_Token("OK", 3000)) {
		mqtt_connected = 0; // BẮT BỆNH ZOMBIE
		return;
	}

	snprintf(at_cmd, sizeof(at_cmd), "AT+CMQTTPAYLOAD=0,%u\r\n",
			(unsigned int) strlen(payload));
	if (!Send_AT_Wait_Prompt_Retry(at_cmd, payload, 2, 2000)
			|| !SIM_Wait_For_Token("OK", 3000)) {
		mqtt_connected = 0; // BẮT BỆNH ZOMBIE
		return;
	}

	if (!Send_AT_With_Retry("AT+CMQTTPUB=0,0,60\r\n", "OK", 2, 2000)) {
		mqtt_connected = 0; // BẮT BỆNH ZOMBIE
	}
}

/* Recovery kết hợp 9 xung clock và trình tự workaround BUSY của STM32F1.
 * Không gọi hàm này từ ISR. */
void I2C_BusRecovery(void) {
	uint8_t line_fault = 0;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	HAL_DMA_StateTypeDef dma_state;

	mpu_recovery_active = 1;

	mpu_ok = 0;
	mpu_sample_valid = 0;
	mpu_sensor_armed = 0;
	mpu_clean_sample_streak = 0;
	mpu_dma_frame_ready = 0;
	mpu_dma_error_pending = 0;
	mpu_low_g_candidate = 0;

	/* Không chỉ dựa vào cờ phần mềm: timeout có thể xảy ra trong khi DMA/HAL
	 * vẫn còn BUSY. Dừng DMA thật trước khi giành lại PB6/PB7. */
	if (hi2c1.hdmarx != NULL) {
		dma_state = HAL_DMA_GetState(hi2c1.hdmarx);
		if (dma_state != HAL_DMA_STATE_READY
				&& dma_state != HAL_DMA_STATE_RESET)
			(void) HAL_DMA_Abort(hi2c1.hdmarx);
	}
	mpu_dma_busy = 0;
	HAL_I2C_DeInit(&hi2c1);
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

	/* PB6=SCL, PB7=SDA theo MSP đã kiểm tra. */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* Release cả hai đường; các điện trở kéo lên trên module phải đưa chúng lên 1. */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(1);
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_SET)
		line_fault = 1;

	/* Slave giữ SDA thấp: phát tối đa 9 clock để nó nhả byte dang dở. */
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
		for (uint8_t i = 0; i < 9U; i++) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_Delay(1);
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET)
				break;
		}
	}

	/* Workaround STM32F1 errata: GPIO tạo START rồi STOP có kiểm tra mức. */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(1);
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_RESET)
		line_fault = 1;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_Delay(1);
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_RESET)
		line_fault = 1;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_Delay(1);
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_SET)
		line_fault = 1;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(1);
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_SET)
		line_fault = 1;

	if (line_fault)
		i2c_recovery_line_fault_count++;

	/* Trả pin về Alternate Function Open Drain trước khi reset peripheral. */
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	SET_BIT(I2C1->CR1, I2C_CR1_SWRST);
	__NOP();
	CLEAR_BIT(I2C1->CR1, I2C_CR1_SWRST);

	MX_I2C1_Init();
	HAL_Delay(5);
	MPU6050_Init();

	/* Loại tick cũ phát sinh trong lúc recovery và không restart DMA ngay. */
	mpu_read_flag = 0;
	mpu_dma_retry_after_tick = HAL_GetTick() + MPU_DMA_SOFT_RETRY_MS;
	mpu_recovery_active = 0;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_IWDG_Init();
	MX_TIM2_Init();
#if ENABLE_SIM
	MX_USART2_UART_Init();
#endif
	MX_ADC1_Init();
	/* STM32F1 cần hiệu chuẩn ADC sau khi khởi tạo để giảm sai số offset. */
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN 2 */
	Load_Phone_From_Flash();

	MPU6050_Init();
	/* Báo một lần để test MPU mà không cần bật log UART blocking liên tục. */
	{
		char mpu_boot_msg[132];
		int len =
				snprintf(mpu_boot_msg, sizeof(mpu_boot_msg),
						"\r\n[MPU] WHO=%02X ST=%u RB=%02X ADDR=%02X OK=%u I2C=%lu CLK=%lu\r\n",
						(unsigned int) mpu_who_am_i,
						(unsigned int) mpu_init_stage,
						(unsigned int) mpu_init_readback,
						(unsigned int) MPU6050_ADDR_ACTIVE,
						(unsigned int) mpu_ok,
						(unsigned long) mpu_last_hal_error,
						(unsigned long) HAL_RCC_GetHCLKFreq());
		if (len > 0) {
			uint16_t tx_len = (uint16_t) (
					(len < (int) sizeof(mpu_boot_msg)) ?
							len : ((int) sizeof(mpu_boot_msg) - 1));
			HAL_UART_Transmit(&huart1, (uint8_t*) mpu_boot_msg, tx_len, 100);
		}
	}
	/* I2C EV/ER và DMA phải cao hơn TIM2/UART để DMA kết thúc đúng hạn. */
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 1);
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 2);
	HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
#if ENABLE_SIM
	HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
#endif
	if (mpu_ok)
		(void) MPU6050_DMA_Start();
	HAL_TIM_Base_Start_IT(&htim2);
#if ENABLE_SIM
	if (HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx3_byte, 1U) != HAL_OK)
		uart2_restart_pending = 1U;
#endif
	Delay_With_Sensor_Service(1000);

#if ENABLE_SIM
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	Delay_With_Sensor_Service(1500);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	HAL_UART_Transmit(&huart1,
			(uint8_t*) "\r\n[SYS] Cho 10s de bo nho SIM khoi dong...\r\n", 44,
			100);
	for (int i = 0; i < 10; i++) {
		Delay_With_Sensor_Service(1000);
	}

	SIM7680_Network_Init();
	SIM7680_GNSS_Init();

	// Chia nhỏ 15 giây thành 150 vòng 100ms để không bị mù AI
	for (int i = 0; i < 150; i++) {
		if (fall_detected_flag || panic_button_flag)
			break; // Cắt ngang nếu có tai nạn
		if (mpu_read_flag == 1) {
			mpu_read_flag = 0;
			Fall_Detection_Task();
		}
		Delay_With_Sensor_Service(100);
	}
	SIM7680_MQTT_Init();
	/* Nhận retained SĐT, gửi pin/vị trí khởi động và giữ RF hoạt động. */
	Delay_With_Sensor_Service(SIM_CONFIG_RX_WINDOW_MS);
	Update_GPS_Location();
	battery_percent = Read_Battery_Percent();
	if (mqtt_connected)
		SIM7680_Publish_Telemetry();
	last_telemetry_tick = HAL_GetTick();
#else
	HAL_UART_Transmit(&huart1, (uint8_t*) "\r\n[SYS] SIM DISABLED\r\n",
			(uint16_t) (sizeof("\r\n[SYS] SIM DISABLED\r\n") - 1U), 100);
#endif

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	Delay_With_Sensor_Service(200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		Poll_Panic_Button();

#if ENABLE_SIM
		UART2_RX_Service();
#endif

		// 1. CHẠY THUẬT TOÁN TÉ NGÃ NGAY CẢ KHI BÌNH THƯỜNG
		if (mpu_read_flag == 1) {
			mpu_read_flag = 0;
			Fall_Detection_Task();
		}

		// 2. GHI FLASH SĐT KHÔNG LÀM KẸT MẠNG (FIX 6)
		if (new_phone_pending_flag) {
			uint32_t now = HAL_GetTick();
			if (!Phone_Is_Valid(pending_phone_number)
					|| strcmp(phone_number, pending_phone_number) == 0) {
				new_phone_pending_flag = 0;
			} else if (!phone_flash_written_this_boot
					|| now - last_phone_flash_write_tick
							>= PHONE_FLASH_MIN_INTERVAL_MS) {
				/* Rate-limit cả lần ghi thất bại để tránh vòng lặp phá Flash. */
				last_phone_flash_write_tick = now;
				phone_flash_written_this_boot = 1;
				if (Save_Phone_To_Flash(pending_phone_number)) {
					strncpy(phone_number, pending_phone_number,
							sizeof(phone_number) - 1U);
					phone_number[sizeof(phone_number) - 1U] = '\0';
					new_phone_pending_flag = 0;
					HAL_UART_Transmit(&huart1,
							(uint8_t*) "\r\n[SYS] DA LUU SDT MOI VAO FLASH\r\n",
							34, 100);
				}
			}
		}

		// 4. XỬ LÝ DỮ LIỆU LIÊN LẠC UART2
#if ENABLE_SIM
		Process_UART2_Data();
#endif

#if ENABLE_VERBOSE_DEBUG
		/* Dùng chung để health log không phát liền kề debug log trong một tick. */
		static uint32_t last_print = 0;
#endif

#if ENABLE_HEALTH_LOG
		/* Log ngắn vẫn hoạt động khi ENABLE_VERBOSE_DEBUG=0. */
		static uint32_t last_health_tick = 0;
		if (HAL_GetTick() - last_health_tick >= 10000U) {
			char health[128];
			int len = snprintf(health, sizeof(health),
					"\r\n[H] Arm:%u MPU:%u DMA:%u I2CF:%lu Miss:%lu Sat:%lu Low:%lu LF:%lu UO:%lu Cell:%u\r\n",
					(unsigned int) mpu_sensor_armed, (unsigned int) mpu_ok,
					(unsigned int) mpu_dma_busy,
					(unsigned long) mpu_read_failure_count,
					(unsigned long) mpu_missed_deadline_count,
					(unsigned long) mpu_rejected_saturation_count,
					(unsigned long) mpu_low_g_candidate_count,
					(unsigned long) i2c_recovery_line_fault_count,
					(unsigned long) rx3_overflow_count,
					(unsigned int) ENABLE_SIM);
			if (len > 0) {
				uint16_t tx_len = (uint16_t) ((len < (int) sizeof(health))
						? len : ((int) sizeof(health) - 1));
				HAL_UART_Transmit(&huart1, (uint8_t*) health, tx_len, 100);
			}
			last_health_tick = HAL_GetTick();
#if ENABLE_VERBOSE_DEBUG
			/* Hoãn debug 2 giây để hai chuỗi UART không cộng dồn quá 10 ms. */
			last_print = last_health_tick;
#endif
		}
#endif

		// DEBUG GỌN: giữ thời gian TX dưới chu kỳ lấy mẫu 10 ms ở 115200 baud.
#if ENABLE_VERBOSE_DEBUG
		if (HAL_GetTick() - last_print > 2000) {
			float tilt_deg = AI_Calculate_Tilt(Ax, Ay, Az);
			char raw_text[16], smooth_text[16], tilt_text[16], score_text[20];

			Format_Fixed(raw_text, sizeof(raw_text), SVM_raw, 2U);
			Format_Fixed(smooth_text, sizeof(smooth_text), SVM_smooth, 2U);
			Format_Fixed(tilt_text, sizeof(tilt_text), tilt_deg, 1U);
			Format_Fixed(score_text, sizeof(score_text), ai_last_score, 2U);

			char debug_buf[144];
			snprintf(debug_buf, sizeof(debug_buf),
					"R:%s S:%s C:%u T:%s V:%u XYZ:%d,%d,%d I:%lu M:%lu I2CF:%lu AI:%s\r\n",
					raw_text, smooth_text, (unsigned int) fall_state, tilt_text,
					(unsigned int) mpu_sample_valid,
					Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW,
					(unsigned long) mpu_last_hal_error,
					(unsigned long) mpu_missed_deadline_count,
					(unsigned long) mpu_read_failure_count,
					score_text);

			HAL_UART_Transmit(&huart1, (uint8_t*) debug_buf, strlen(debug_buf),
					100);
			last_print = HAL_GetTick();
		}
#endif

#if ENABLE_RING_LOG
		/* Luôn in lại bốn sự kiện gần nhất. UART không biết PuTTY đang mở hay
		 * không, nên việc lặp lại giúp cắm UART sau vẫn đọc được lịch sử RAM. */
		static uint32_t next_dump_tick = 5000U;
		if ((int32_t) (HAL_GetTick() - next_dump_tick) >= 0) {
			next_dump_tick = HAL_GetTick() + 10000U;
			uint8_t stored_events = (event_log_seq < EVENT_LOG_SIZE)
					? (uint8_t) event_log_seq : EVENT_LOG_SIZE;
			uint8_t printed_events = (stored_events > 4U) ? 4U : stored_events;

			if (printed_events > 0U) {
				HAL_UART_Transmit(&huart1,
						(uint8_t*) "\r\n--- LICH SU GAN DAY ---\r\n", 27,
						100);
			}

			/* event_log_idx trỏ tới ô sẽ ghi tiếp theo. Lùi printed_events ô rồi
			 * đi tới để lịch sử được hiển thị theo thứ tự cũ -> mới. */
			for (uint8_t i = 0; i < printed_events; i++) {
				int idx = (event_log_idx + EVENT_LOG_SIZE - printed_events + i)
						% EVENT_LOG_SIZE;
				char line[90];
				char raw_text[16];
				Format_Fixed(raw_text, sizeof(raw_text), event_log[idx].raw, 2U);
				snprintf(line, sizeof(line), "[t=%lus] %s %d->%d Raw:%s\r\n",
						(unsigned long) (event_log[idx].t / 1000U),
						event_log[idx].type == 1 ? "I2C-LOI" : "STATE",
						event_log[idx].from_s, event_log[idx].to_s,
						raw_text);
				HAL_UART_Transmit(&huart1, (uint8_t*) line, strlen(line), 100);
			}
		}
#endif

		// ====================================================
		// 5. LOGIC TIỀN CẢNH BÁO 10S CHO CẢ SOS LẪN TÉ NGÃ (FIX 4)
		if (panic_button_flag == 1 || fall_detected_flag == 1) {

			if (!pre_alarm_active) {
				pre_alarm_active = 1;
				pre_alarm_start_time = HAL_GetTick();
				if (panic_button_flag == 1)
					strcpy(alarm_reason_str, "NGUOI DUNG NHAN NUT KHAN CAP");
				else
					strcpy(alarm_reason_str, "PHAT HIEN TE NGA");

			} else if (panic_button_flag == 1) {
				// CHỈ CHO PHÉP NÚT BẤM (panic_button) MỚI ĐƯỢC QUYỀN HỦY BÁO ĐỘNG
				pre_alarm_active = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_UART_Transmit(&huart1,
						(uint8_t*) "\r\n[SYS] NGUOI DUNG HUY BAO DONG\r\n", 33,
						100);
			}

			// Xóa cờ để chuẩn bị cho chu kỳ tiếp theo
			panic_button_flag = 0;
			fall_detected_flag = 0;

			/* Debounce/edge detection đã được Poll_Panic_Button xử lý non-blocking. */
		}

		// Vận hành còi/LED 10s trước khi gửi tin nhắn thực sự
		if (pre_alarm_active) {
			if ((HAL_GetTick() / 200) % 2 == 0)
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

			if (HAL_GetTick() - pre_alarm_start_time > 10000) {
				pre_alarm_active = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				Trigger_Alarm(alarm_reason_str);
			}
		}

#if ENABLE_SIM
		/* Đồng bộ nền khi state machine rảnh. telemetry_interval đã tự chọn
		 * 5 phút khi MOVING và 30 phút khi STILL, vì vậy không được chặn
		 * is_moving tại đây nếu muốn App nhận đúng trạng thái di chuyển. */
		if (fall_detected_flag == 0 && pre_alarm_active == 0 && mpu_sensor_armed
				&& fall_state == 0
				&& HAL_GetTick() - last_telemetry_tick >= telemetry_interval) {
			Update_GPS_Location();
			battery_percent = Read_Battery_Percent();

			SIM7680_MQTT_Init();
			Delay_With_Sensor_Service(SIM_CONFIG_RX_WINDOW_MS);
			if (mqtt_connected) {
				SIM7680_Publish_Telemetry();
				HAL_UART_Transmit(&huart1,
						(uint8_t*) "\r\n[SYS] DONG BO APP XONG\r\n",
						(uint16_t) (sizeof("\r\n[SYS] DONG BO APP XONG\r\n")
								- 1U), 100);
			}
			last_telemetry_tick = HAL_GetTick();
		}
#endif

		// 7. CHẾ ĐỘ NGỦ TIẾT KIỆM PIN
		HAL_IWDG_Refresh(&hiwdg);
		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * Enable DMA controller clock and DMA1 Channel7 interrupt for I2C1_RX.
 * The channel parameters and __HAL_LINKDMA are generated in HAL_I2C_MspInit.
 */
static void MX_DMA_Init(void) {
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 2);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	/* Cầu chia áp 10k-10k có trở kháng nguồn tương đối cao; 55.5 chu kỳ cho
	 * tụ sample-and-hold đủ thời gian nạp và làm số đo pin ổn định hơn. */
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 50000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 36000 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA4 PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t Read_Battery_Percent(void) {
	uint32_t sum = 0;
	uint8_t valid_samples = 0;

	// Lấy 10 mẫu liên tiếp tốc độ cao để lọc nhiễu ngẫu nhiên
	for (int i = 0; i < 10; i++) {
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
			sum += HAL_ADC_GetValue(&hadc1);
			valid_samples++;
		}
		HAL_ADC_Stop(&hadc1);
	}

	// Nếu không đọc được mẫu nào hợp lệ thì trả về giá trị pin cũ
	if (valid_samples == 0) {
		return battery_percent;
	}

	// 1. Tính trung bình ADC và đổi ra điện áp chân PA0 (Đã fix lỗi ép kiểu)
	float v_pin = ((float) sum / (float) valid_samples / 4095.0f) * 3.3f;

	// 2. Nhân 2 vì đã qua cầu phân áp 10k-10k
	float v_batt = v_pin * 2.0f;

	// 3. Quy đổi % (Tuyến tính từ 3.2V đến 4.2V)
	float percent = ((v_batt - 3.2f) / (4.2f - 3.2f)) * 100.0f;

	// 4. Giới hạn trong dải an toàn (0% - 100%)
	if (percent > 100.0f)
		percent = 100.0f;
	if (percent < 0.0f)
		percent = 0.0f;

	return (uint8_t) percent;
}

/* ISR chỉ bàn giao trạng thái; parse frame, recovery và AI đều chạy ở main. */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		mpu_dma_busy = 0;
		mpu_dma_frame_ready = 1;
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		mpu_dma_error_code = HAL_I2C_GetError(hi2c);
		mpu_dma_busy = 0;
		mpu_dma_frame_ready = 0;
		mpu_dma_error_pending = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
#if ENABLE_SIM
	if (huart->Instance == USART2) {
		uint16_t next_head = (rx3_head + 1) % RX3_BUF_SIZE;
		if (next_head != rx3_tail) {
			rx3_buf[rx3_head] = rx3_byte;
			rx3_head = next_head;
		} else {
			rx3_overflow_count++;
			rx3_desync = 1;
		}
		if (HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx3_byte, 1U) != HAL_OK)
			uart2_restart_pending = 1U;
	}
#else
	(void) huart;
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
#if ENABLE_SIM
	if (huart->Instance == USART2) {
		rx3_desync = 1;
		uart2_restart_pending = 1U;
	}
#else
	(void) huart;
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2 && !mpu_recovery_active) {
		if (mpu_read_flag)
			mpu_missed_deadline_count++;
		else
			mpu_read_flag = 1;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

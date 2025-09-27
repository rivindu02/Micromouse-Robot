// Create sensors.c - IR sensors and MPU9250 implementation

/*
 * sensors.c
 *
 * IR sensor (SFH4545 + TEFT4300) and MPU9250 implementation
 * Handles wall detection and robot orientation
 * Handles ADC reading for IR sensors and battery monitoring
 *
 * Author: Micromouse v1.0
 * Date: 2025
 */

#include "micromouse.h"
#include "core_cm4.h"  // F411 = Cortex-M4

// sensors.c
#define EMITTER_ACTIVE_LOW  0   // set to 1 if your drivers are active-low

#if EMITTER_ACTIVE_LOW
  #define EMIT_ON(port,pin)   HAL_GPIO_WritePin((port),(pin),GPIO_PIN_RESET)
  #define EMIT_OFF(port,pin)  HAL_GPIO_WritePin((port),(pin),GPIO_PIN_SET)
#else
  #define EMIT_ON(port,pin)   HAL_GPIO_WritePin((port),(pin),GPIO_PIN_SET)
  #define EMIT_OFF(port,pin)  HAL_GPIO_WritePin((port),(pin),GPIO_PIN_RESET)
#endif

// Calibration values are the raw reading from the sensor
// NOTE: side sensors see the front wall when the mouse is centered
#define LD_CAL 624
#define RD_CAL 635
// NOTE: front sensor calibration is with the mouse against the rear wall
#define LF_CAL 602
#define RF_CAL 622

// values that the sensors get normalised to when the mouse is correctly positioned
// defined as longs to prevent overflow when normalising
#define LD_NOMINAL 100L
#define RD_NOMINAL 100L
#define LF_NOMINAL 100L
#define RF_NOMINAL 100L




// Calibration data structure
typedef struct {
    uint16_t ambient_baseline[4];        // Baseline ambient light for each sensor
    uint16_t wall_thresholds[4];         // Dynamic wall detection thresholds
    uint16_t battery_baseline;           // Battery voltage baseline
    uint16_t sensor_min[4];             // Minimum readings during calibration
    uint16_t sensor_max[4];             // Maximum readings during calibration
    bool calibration_valid;              // Flag indicating successful calibration
    float noise_levels[4];              // Noise level for each sensor
} SensorCalibration;

static SensorCalibration sensor_cal = {0};
/**
 * @brief Turn on IR emitters
 */
void turn_on_emitters(void)
{
    HAL_GPIO_WritePin(EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EMIT_SIDE_LEFT_GPIO_Port, EMIT_SIDE_LEFT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EMIT_SIDE_RIGHT_GPIO_Port, EMIT_SIDE_RIGHT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EMIT_FRONT_RIGHT_GPIO_Port, EMIT_FRONT_RIGHT_Pin, GPIO_PIN_SET);
    dwt_delay_us(200); // small settle (µs), not HAL_Delay(ms); // Emitter stabilization time
}

/**
 * @brief Turn off IR emitters
 */
void turn_off_emitters(void)
{
    HAL_GPIO_WritePin(EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EMIT_SIDE_LEFT_GPIO_Port, EMIT_SIDE_LEFT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EMIT_SIDE_RIGHT_GPIO_Port, EMIT_SIDE_RIGHT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EMIT_FRONT_RIGHT_GPIO_Port, EMIT_FRONT_RIGHT_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Read specific ADC channel using main.c multi-channel setup
 */
uint16_t read_adc_channel(uint32_t channel) {
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel = channel;
    cfg.Rank = 1;
    cfg.SamplingTime = ADC_SAMPLETIME_480CYCLES; // more stable than 84

    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) return 0;

    dwt_delay_us(5);                     // tiny mux settle

    // dummy conversion (discard)
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    (void)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    // real conversion
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t v = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}


// ===== sensors_sync.c additions =====
// Deterministic microsecond delay using DWT (Cortex-M3/M4/M7).
// Call dwt_delay_init() once at startup (e.g., in main()).

// sensors.c


static uint32_t dwt_cycles_per_us;

void dwt_delay_init(uint32_t cpu_hz) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_cycles_per_us = cpu_hz / 1000000U; // e.g., 84 for 84 MHz
}

static inline void dwt_delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * dwt_cycles_per_us;
    while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}


// ---- Tunables (typical values from micromouse practice) ----
#define IR_CYCLES   32     // average of 12 ON/OFF pairs per sensor
#define T_OFF_US    80     // wait after LED OFF before sample
#define T_ON_US     120     // wait after LED ON before sample

static inline uint16_t diff_once(GPIO_TypeDef* port, uint16_t pin, uint32_t ch) {
    // OFF sample
    EMIT_OFF(port, pin);
    dwt_delay_us(T_OFF_US);
    uint16_t offv = read_adc_channel(ch);

    // ON sample
    EMIT_ON(port, pin);
    dwt_delay_us(T_ON_US);
    uint16_t onv  = read_adc_channel(ch);

    EMIT_OFF(port, pin);

    int32_t d = (int32_t)onv - (int32_t)offv;
    return (d > 0) ? (uint16_t)d : 0;
}

static uint16_t measure_sync(GPIO_TypeDef* port, uint16_t pin, uint32_t ch) {
    uint32_t acc = 0;
    for (int k = 0; k < IR_CYCLES; k++) {
        acc += diff_once(port, pin, ch);
        dwt_delay_us(20);
    }
    if (acc > 4095) acc = 4095;   // clamp to 12-bit domain if you prefer
    return (uint16_t)acc;
}

int point=0;

uint32_t FL_buff[5];
uint32_t FR_buff[5];
uint32_t L_buff[5];
uint32_t R_buff[5];

void update_sensors(void){
	turn_off_emitters();
	//dwt_delay_us(500);
	uint16_t off_FL = read_adc_channel(ADC_CHANNEL_5);
	uint16_t off_FR = read_adc_channel(ADC_CHANNEL_2);
	uint16_t off_L = read_adc_channel(ADC_CHANNEL_4);
	uint16_t off_R = read_adc_channel(ADC_CHANNEL_3);

	EMIT_ON(EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_LEFT_Pin);
	EMIT_ON(EMIT_FRONT_RIGHT_GPIO_Port, EMIT_FRONT_RIGHT_Pin);
	dwt_delay_us(500);
	//HAL_Delay(1);

	uint16_t on_FL = read_adc_channel(ADC_CHANNEL_5);
	uint16_t on_FR = read_adc_channel(ADC_CHANNEL_2);

	turn_off_emitters();
	EMIT_ON(EMIT_SIDE_LEFT_GPIO_Port, EMIT_SIDE_LEFT_Pin);
	EMIT_ON(EMIT_SIDE_RIGHT_GPIO_Port, EMIT_SIDE_RIGHT_Pin);
	dwt_delay_us(800);
	//HAL_Delay(1);

	uint16_t on_L = read_adc_channel(ADC_CHANNEL_4);
	uint16_t on_R = read_adc_channel(ADC_CHANNEL_3);

	turn_off_emitters();

	uint32_t diff_FL;
	uint32_t diff_FR;
	uint32_t diff_L;
	uint32_t diff_R;

	if (on_FL>off_FL){
		diff_FL = (uint32_t)on_FL-(uint32_t)off_FL;
	}else{
		diff_FL =0;
	}
	if (on_FR>off_FR){
		diff_FR = (uint32_t)on_FR-(uint32_t)off_FR;
	}else{
		diff_FR =0;
	}
	if (on_L>off_L){
		diff_L = (uint32_t)on_L-(uint32_t)off_L;
	}else{
		diff_L=0;
	}
	if (on_R>off_R){
		diff_R = (uint32_t)on_R-(uint32_t)off_R;
	}else{
		diff_R = 0;
	}

	if (point>=5) point=0;



	diff_FL=(diff_FL*NOMINAL)/1000;
	diff_FR=(diff_FR*NOMINAL)/1000;
	diff_L=(diff_L*NOMINAL)/1000;
	diff_R=(diff_R*NOMINAL)/1000;




	FL_buff[point]=diff_FL;
	FR_buff[point]=diff_FR;
	L_buff[point]=diff_L;
	R_buff[point]=diff_R;

	point++;

	uint32_t tot_diff_FL=0;
	uint32_t tot_diff_FR=0;
	uint32_t tot_diff_L=0;
	uint32_t tot_diff_R=0;

	for (int i=0;i<5;i++){
		tot_diff_FL+=FL_buff[i];
		tot_diff_FR+=FR_buff[i];
		tot_diff_L+=L_buff[i];
		tot_diff_R+=R_buff[i];
	}

    sensors.front_left  = tot_diff_FL/5; //diff_FL; //
    sensors.front_right = tot_diff_FR/5; //diff_FR; //
    sensors.side_left   = tot_diff_L/5;  //diff_L; //
    sensors.side_right  = tot_diff_R/5;  //diff_R; //
    sensors.battery = read_adc_channel(ADC_CHANNEL_0);

    // Process wall detection using calibrated thresholds
    if (sensor_cal.calibration_valid) {
        // Use dynamic thresholds
        sensors.wall_front = (sensors.front_left > get_calibrated_threshold(0)) ||
                            (sensors.front_right > get_calibrated_threshold(1));
        sensors.wall_left = (sensors.side_left > get_calibrated_threshold(2));
        sensors.wall_right = (sensors.side_right > get_calibrated_threshold(3));
    } else {
        // Fallback to static thresholds
        sensors.wall_front = (sensors.front_left > WALL_THRESHOLD_FRONT) ||
                            (sensors.front_right > WALL_THRESHOLD_FRONT);
        sensors.wall_left = (sensors.side_left > WALL_THRESHOLD_SIDE);
        sensors.wall_right = (sensors.side_right > WALL_THRESHOLD_SIDE);
    }


//	send_bluetooth_printf("FL:%u - %u  FR:%u - %u  SL:%u - %u  SR:%u - %u \r\n",
//	                          on_FL, off_FL, on_FR,off_FR,
//	                          on_L, off_L,on_R, off_R);


	send_bluetooth_printf("FL:%u   FR:%u  Fwall: %d SL:%u Lwall: %d  SR:%u  Rwall: %d  \r\n",
		                          sensors.front_left, sensors.front_right,sensors.wall_front,
		                          sensors.side_left, sensors.wall_left, sensors.side_right, sensors.wall_right);

}




void update_sensors3(void)
{
    uint32_t ch[4]    = {ADC_CHANNEL_5, ADC_CHANNEL_2, ADC_CHANNEL_4, ADC_CHANNEL_3};
    GPIO_TypeDef* p[4]= {EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_RIGHT_GPIO_Port,
                         EMIT_SIDE_LEFT_GPIO_Port,  EMIT_SIDE_RIGHT_GPIO_Port};
    uint16_t pin[4]   = {EMIT_FRONT_LEFT_Pin, EMIT_FRONT_RIGHT_Pin,
                         EMIT_SIDE_LEFT_Pin,  EMIT_SIDE_RIGHT_Pin};

    // synchronous, ambient-rejected readings
    uint16_t fl = measure_sync(p[0], pin[0], ch[0]);
    uint16_t fr = measure_sync(p[1], pin[1], ch[1]);
    uint16_t sl = measure_sync(p[2], pin[2], ch[2]);
    uint16_t sr = measure_sync(p[3], pin[3], ch[3]);

    sensors.front_left  = fl;
    sensors.front_right = fr;
    sensors.side_left   = sl;
    sensors.side_right  = sr;

    // battery (light average)
    uint32_t bat=0; for (int i=0;i<8;i++) bat += read_adc_channel(ADC_CHANNEL_0);
    sensors.battery = (uint16_t)(bat/8);

    // thresholds (use your calibrated ones if valid)
    uint16_t th_fl = is_sensor_calibration_valid() ? get_calibrated_threshold(0) : WALL_THRESHOLD_FRONT;
    uint16_t th_fr = is_sensor_calibration_valid() ? get_calibrated_threshold(1) : WALL_THRESHOLD_FRONT;
    uint16_t th_sl = is_sensor_calibration_valid() ? get_calibrated_threshold(2) : WALL_THRESHOLD_SIDE;
    uint16_t th_sr = is_sensor_calibration_valid() ? get_calibrated_threshold(3) : WALL_THRESHOLD_SIDE;

    sensors.wall_front = (fl > th_fl) || (fr > th_fr);
    sensors.wall_left  = (sl > th_sl);
    sensors.wall_right = (sr > th_sr);

	send_bluetooth_printf("FL:%u   FR:%u   SL:%u   SR:%u    \r\n",
		                          sensors.front_left, sensors.front_right,
		                          sensors.side_left, sensors.side_right);

}




/**
 * @brief Enhanced update_sensors with calibrated thresholds
 */
void update_sensors2(void)
{
//    // Read ambient light levels (emitters off)
//    turn_off_emitters();
//    HAL_Delay(5);
//    uint16_t ambient_front_right = read_adc_channel(ADC_CHANNEL_2);
//    uint16_t ambient_side_right = read_adc_channel(ADC_CHANNEL_3);
//    uint16_t ambient_side_left = read_adc_channel(ADC_CHANNEL_4);
//    uint16_t ambient_front_left = read_adc_channel(ADC_CHANNEL_5);
//
//    // Read with emitters on
//    turn_on_emitters();
//    sensors.battery = read_adc_channel(ADC_CHANNEL_0);
//    sensors.front_right = read_adc_channel(ADC_CHANNEL_2) - ambient_front_right;
//    sensors.side_right = read_adc_channel(ADC_CHANNEL_3) - ambient_side_right;
//    sensors.side_left = read_adc_channel(ADC_CHANNEL_4) - ambient_side_left;
//    sensors.front_left = read_adc_channel(ADC_CHANNEL_5) - ambient_front_left;
//
//    // Turn off emitters to save power
//    turn_off_emitters();

	uint32_t channels[] = {ADC_CHANNEL_5, ADC_CHANNEL_2, ADC_CHANNEL_4, ADC_CHANNEL_3};
	GPIO_TypeDef* emit_ports[] = {EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_RIGHT_GPIO_Port,
								  EMIT_SIDE_LEFT_GPIO_Port, EMIT_SIDE_RIGHT_GPIO_Port};
	uint16_t emit_pins[] = {EMIT_FRONT_LEFT_Pin, EMIT_FRONT_RIGHT_Pin,
						   EMIT_SIDE_LEFT_Pin, EMIT_SIDE_RIGHT_Pin};

	int16_t difference[4];



	turn_off_emitters();

	for(int i = 0; i < 4; i++) {
		// Test with emitter OFF
		HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_RESET);
		HAL_Delay(2);	//10
		uint16_t off_reading = read_adc_channel(channels[i]);

		// Test with emitter ON
		HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_SET);
		HAL_Delay(2);	//10
		uint16_t on_reading = read_adc_channel(channels[i]);


		// Calculate difference
		//difference[i] = on_reading - off_reading;

		// Calculate difference (clamped to >= 0)
		int32_t d = (int32_t)on_reading - (int32_t)off_reading;
		if (d < 0) d = 0;
		difference[i] = (int16_t)d;


		// Turn off emitter
		HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_RESET);
		HAL_Delay(3);	//50
	}
	sensors.battery = read_adc_channel(ADC_CHANNEL_0);
	sensors.front_right = difference[1];
	sensors.side_right = difference[3];
	sensors.side_left = difference[2];
	sensors.front_left = difference[0];

    // Process wall detection using calibrated thresholds
    if (sensor_cal.calibration_valid) {
        // Use dynamic thresholds
        sensors.wall_front = (sensors.front_left > get_calibrated_threshold(0)) ||
                            (sensors.front_right > get_calibrated_threshold(1));
        sensors.wall_left = (sensors.side_left > get_calibrated_threshold(2));
        sensors.wall_right = (sensors.side_right > get_calibrated_threshold(3));
    } else {
        // Fallback to static thresholds
        sensors.wall_front = (sensors.front_left > WALL_THRESHOLD_FRONT) ||
                            (sensors.front_right > WALL_THRESHOLD_FRONT);
        sensors.wall_left = (sensors.side_left > WALL_THRESHOLD_SIDE);
        sensors.wall_right = (sensors.side_right > WALL_THRESHOLD_SIDE);
    }

    // Enhanced sensor health monitoring using calibration data
    static uint8_t sensor_error_count = 0;

    if (sensor_cal.calibration_valid) {
        // Check if readings are within expected ranges based on calibration
        bool current_reading_valid = (sensors.battery > sensor_cal.battery_baseline - 200) &&
                                   (sensors.battery < sensor_cal.battery_baseline + 500) &&
                                   (sensors.front_left < 3500) &&
                                   (sensors.front_right < 3500) &&
                                   (sensors.side_left < 3500) &&
                                   (sensors.side_right < 3500);

        if (!current_reading_valid) {
            sensor_error_count++;

        } else {
            if (sensor_error_count > 0) sensor_error_count--; // Recover slowly
        }
    }
}

/**
 * @brief Update maze walls based on sensor readings
 */
void update_walls(void)
{
    // Update walls based on current direction and sensor readings
    if (sensors.wall_front) {
        maze[robot.x][robot.y].walls[robot.direction] = true;
        // Update opposite wall in neighbor cell
        int nx = robot.x + dx[robot.direction];
        int ny = robot.y + dy[robot.direction];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            maze[nx][ny].walls[(robot.direction + 2) % 4] = true;
        }
    }

    if (sensors.wall_left) {
        int left_dir = (robot.direction + 3) % 4;
        maze[robot.x][robot.y].walls[left_dir] = true;
        int nx = robot.x + dx[left_dir];
        int ny = robot.y + dy[left_dir];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            maze[nx][ny].walls[(left_dir + 2) % 4] = true;
        }
    }

    if (sensors.wall_right) {
        int right_dir = (robot.direction + 1) % 4;
        maze[robot.x][robot.y].walls[right_dir] = true;
        int nx = robot.x + dx[right_dir];
        int ny = robot.y + dy[right_dir];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
            maze[nx][ny].walls[(right_dir + 2) % 4] = true;
        }
    }
    // Simple debug feedback
    if (sensors.wall_front || sensors.wall_left || sensors.wall_right) {
        send_bluetooth_printf("Walls: F:%s L:%s R:%s [FL:%d FR:%d SL:%d SR:%d]\r\n",
            sensors.wall_front ? "Y" : "N",
            sensors.wall_left ? "Y" : "N",
            sensors.wall_right ? "Y" : "N",
            sensors.front_left, sensors.front_right,
            sensors.side_left, sensors.side_right);
        play_wall_beep();
    }


    // Mark current cell as visited
    maze[robot.x][robot.y].visited = true;
    maze[robot.x][robot.y].visit_count++;
}


/**
 * @brief Get sensor calibration status
 */
bool is_sensor_calibration_valid(void)
{
    return sensor_cal.calibration_valid;
}

/**
 * @brief Send detailed sensor status including calibration data
 */
void send_detailed_sensor_status(void)
{
    send_bluetooth_message("\r\n=== DETAILED SENSOR STATUS ===\r\n");

    if (sensor_cal.calibration_valid) {
        send_bluetooth_message("Calibration: ✅ VALID\r\n");

        // Update sensors first
        update_sensors();

        send_bluetooth_message("Current readings vs baselines:\r\n");
        send_bluetooth_printf("Front Left (diff):  %u (ambient: %u, th: %u)\r\n",
                             sensors.front_left,
                             sensor_cal.ambient_baseline[0],
                             sensor_cal.wall_thresholds[0]);
        send_bluetooth_printf("Front Right (diff): %u (ambient: %u, th: %u)\r\n",
                             sensors.front_right,
                             sensor_cal.ambient_baseline[1],
                             sensor_cal.wall_thresholds[1]);
        send_bluetooth_printf("Side Left (diff): %u (ambient: %u, th: %u)\r\n",
                             sensors.side_left,
                             sensor_cal.ambient_baseline[2],
                             sensor_cal.wall_thresholds[2]);
        send_bluetooth_printf("Side Right (diff): %u (ambient: %u, th: %u)\r\n",
                             sensors.side_right,
                             sensor_cal.ambient_baseline[3],
                             sensor_cal.wall_thresholds[3]);

        send_bluetooth_printf("Battery: %d (baseline: %d)\r\n",
                             sensors.battery, sensor_cal.battery_baseline);

        send_bluetooth_printf("Wall detection: F:%s L:%s R:%s\r\n",
                             sensors.wall_front ? "YES" : "NO",
                             sensors.wall_left ? "YES" : "NO",
                             sensors.wall_right ? "YES" : "NO");
    } else {
        send_bluetooth_message("Calibration: ❌ INVALID - Using default thresholds\r\n");
    }

    send_bluetooth_message("===============================\r\n");
}


void adc_system_diagnostics(void) {
    send_bluetooth_message("\r\n=== ADC SYSTEM DIAGNOSTICS ===\r\n");

    // Check if ADC clock is enabled
    if (__HAL_RCC_ADC1_IS_CLK_ENABLED()) {
        send_bluetooth_message("✅ ADC1 clock: ENABLED\r\n");
    } else {
        send_bluetooth_message("❌ ADC1 clock: DISABLED\r\n");
    }

    // Check GPIO clock
    if (__HAL_RCC_GPIOA_IS_CLK_ENABLED()) {
        send_bluetooth_message("✅ GPIOA clock: ENABLED\r\n");
    } else {
        send_bluetooth_message("❌ GPIOA clock: DISABLED\r\n");
    }

    // Check ADC status
    if (hadc1.State == HAL_ADC_STATE_READY) {
        send_bluetooth_message("✅ ADC state: READY\r\n");
    } else {
        send_bluetooth_printf("⚠️ ADC state: %d\r\n", hadc1.State);
    }

    // Test individual channel readings
    send_bluetooth_message("Testing individual channels:\r\n");

    uint32_t channels[5] = {ADC_CHANNEL_0, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_5};
    const char* channel_names[5] = {"Battery", "Front_Right", "Side_Right", "Side_Left", "Front_Left"};
    uint16_t test_values[5];  // Fixed: Added array brackets

    for (int i = 0; i < 5; i++) {
        test_values[i] = read_adc_channel(channels[i]);
        send_bluetooth_printf("%s (CH%d): %d\r\n", channel_names[i],
                             channels[i] == ADC_CHANNEL_0 ? 0 :
                             channels[i] == ADC_CHANNEL_2 ? 2 :
                             channels[i] == ADC_CHANNEL_3 ? 3 :
                             channels[i] == ADC_CHANNEL_4 ? 4 : 5, test_values[i]);
    }

    send_bluetooth_message("===============================\r\n");
}

// Add this function to sensors.c for debugging
void diagnostic_sensor_test(void) {
    send_bluetooth_message("\r\n=== IR SENSOR DIAGNOSTIC ===\r\n");

    // Test each emitter-detector pair individually
    const char* sensor_names[] = {"Front Left", "Front Right", "Side Left", "Side Right"};
    uint32_t channels[] = {ADC_CHANNEL_5, ADC_CHANNEL_2, ADC_CHANNEL_4, ADC_CHANNEL_3};
    GPIO_TypeDef* emit_ports[] = {EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_RIGHT_GPIO_Port,
                                  EMIT_SIDE_LEFT_GPIO_Port, EMIT_SIDE_RIGHT_GPIO_Port};
    uint16_t emit_pins[] = {EMIT_FRONT_LEFT_Pin, EMIT_FRONT_RIGHT_Pin,
                           EMIT_SIDE_LEFT_Pin, EMIT_SIDE_RIGHT_Pin};

    for(int i = 0; i < 4; i++) {
        // Test with emitter OFF
        HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_RESET);
        HAL_Delay(10);
        uint16_t off_reading = read_adc_channel(channels[i]);

        // Test with emitter ON
        HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_SET);
        HAL_Delay(10);
        uint16_t on_reading = read_adc_channel(channels[i]);



        // Calculate difference
        int16_t difference = on_reading - off_reading;

        send_bluetooth_printf("%s: OFF=%d, ON=%d, DIFF=%d\r\n",
                             sensor_names[i], off_reading, on_reading, difference);

        // Turn off emitter
        HAL_GPIO_WritePin(emit_ports[i], emit_pins[i], GPIO_PIN_RESET);
        HAL_Delay(50);
    }

    send_bluetooth_message("=== Place hand/object 5cm from sensor and retest ===\r\n");
}


/**
 * @brief Enhanced sensor calibration - replaces main.c ADC test code
 * This function performs comprehensive sensor calibration including:
 * - Baseline ambient light measurement
 * - Dynamic wall threshold calculation
 * - Sensor health validation
 * - Battery voltage baseline
 */
void calibrate_sensors(void)
{
    send_bluetooth_message("\r\n=== ENHANCED SENSOR CALIBRATION ===\r\n");

    // Initialize calibration structure
    memset(&sensor_cal, 0, sizeof(sensor_cal));

    // Phase 1: ADC System Validation
    send_bluetooth_message("Phase 1: ADC System Validation\r\n");

    // Check if ADC is properly initialized
    if (hadc1.State != HAL_ADC_STATE_READY) {
        send_bluetooth_message("❌ ADC not ready - attempting re-initialization\r\n");
        if (HAL_ADC_Init(&hadc1) != HAL_OK) {
            send_bluetooth_message("❌ CRITICAL: ADC initialization failed!\r\n");
            return;
        }
    }

    // Verify clock enables
    if (!__HAL_RCC_ADC1_IS_CLK_ENABLED()) {
        send_bluetooth_message("❌ ADC1 clock disabled\r\n");
        return;
    }

    if (!__HAL_RCC_GPIOA_IS_CLK_ENABLED()) {
        send_bluetooth_message("❌ GPIOA clock disabled\r\n");
        return;
    }

    send_bluetooth_message("✅ ADC system validation passed\r\n");

    // Phase 2: Baseline Ambient Light Measurement
    send_bluetooth_message("Phase 2: Measuring ambient baselines (IR emitters OFF)\r\n");

    // Ensure emitters are OFF
    turn_off_emitters();
    HAL_Delay(100); // Allow sensors to stabilize

    // Take multiple ambient readings for stability
    uint32_t ambient_sum[4] = {0};
    uint32_t ambient_readings = 50;

    for (int i = 0; i < ambient_readings; i++) {
        ambient_sum[0] += read_adc_channel(ADC_CHANNEL_5); // Front Left
        ambient_sum[1] += read_adc_channel(ADC_CHANNEL_2); // Front Right
        ambient_sum[2] += read_adc_channel(ADC_CHANNEL_4); // Side Left
        ambient_sum[3] += read_adc_channel(ADC_CHANNEL_3); // Side Right
        HAL_Delay(10);
    }

    // Calculate ambient baselines
    sensor_cal.ambient_baseline[0] = ambient_sum[0] / ambient_readings; // Front Left
    sensor_cal.ambient_baseline[1] = ambient_sum[1] / ambient_readings; // Front Right
    sensor_cal.ambient_baseline[2] = ambient_sum[2] / ambient_readings; // Side Left
    sensor_cal.ambient_baseline[3] = ambient_sum[3] / ambient_readings; // Side Right

    send_bluetooth_message("Ambient baselines (emitters OFF):\r\n");
    send_bluetooth_printf("  Front Left:  %d\r\n", sensor_cal.ambient_baseline[0]);
    send_bluetooth_printf("  Front Right: %d\r\n", sensor_cal.ambient_baseline[1]);
    send_bluetooth_printf("  Side Left:   %d\r\n", sensor_cal.ambient_baseline[2]);
    send_bluetooth_printf("  Side Right:  %d\r\n", sensor_cal.ambient_baseline[3]);

    // Phase 3: Differential response per sensor (ON-OFF, single emitter only)
    send_bluetooth_message("Phase 3: Measuring differential response per sensor\r\n");

    uint16_t diff_avg[4] = {0};
    float    diff_noise[4] = {0};

    uint32_t ch[] = {ADC_CHANNEL_5, ADC_CHANNEL_2, ADC_CHANNEL_4, ADC_CHANNEL_3};
    GPIO_TypeDef* prt[] = {EMIT_FRONT_LEFT_GPIO_Port, EMIT_FRONT_RIGHT_GPIO_Port,
                           EMIT_SIDE_LEFT_GPIO_Port,  EMIT_SIDE_RIGHT_GPIO_Port};
    uint16_t pin[] = {EMIT_FRONT_LEFT_Pin, EMIT_FRONT_RIGHT_Pin,
                      EMIT_SIDE_LEFT_Pin,  EMIT_SIDE_RIGHT_Pin};

    // make sure all emitters are OFF
    turn_off_emitters();

    for (int i=0;i<4;i++){
        const int K = 20;        // minimal samples for an average
        uint32_t acc = 0;
        uint16_t minv = 0xFFFF, maxv = 0;

        for (int k=0;k<K;k++){
            // OFF window (emitter off)
            HAL_GPIO_WritePin(prt[i], pin[i], GPIO_PIN_RESET);
            uint16_t off = read_adc_channel(ch[i]);

            // ON window (only this emitter on)
            HAL_GPIO_WritePin(prt[i], pin[i], GPIO_PIN_SET);
            uint16_t on  = read_adc_channel(ch[i]);

            // back to OFF
            HAL_GPIO_WritePin(prt[i], pin[i], GPIO_PIN_RESET);

            uint16_t d = (on > off) ? (on - off) : 0;
            acc += d;
            if (d < minv) minv = d;
            if (d > maxv) maxv = d;

            HAL_Delay(2);
        }

        diff_avg[i]            = (uint16_t)(acc / K);
        diff_noise[i]          = (float)(maxv - minv);
        sensor_cal.noise_levels[i] = diff_noise[i];
        sensor_cal.sensor_min[i]   = minv;
        sensor_cal.sensor_max[i]   = maxv;
    }

    send_bluetooth_message("Differential response (ON-OFF):\r\n");
    send_bluetooth_printf("  Front Left:  %u (noise: %.1f)\r\n", diff_avg[0], diff_noise[0]);
    send_bluetooth_printf("  Front Right: %u (noise: %.1f)\r\n", diff_avg[1], diff_noise[1]);
    send_bluetooth_printf("  Side Left:   %u (noise: %.1f)\r\n", diff_avg[2], diff_noise[2]);
    send_bluetooth_printf("  Side Right:  %u (noise: %.1f)\r\n", diff_avg[3], diff_noise[3]);

    // Phase 4: Set DIFFERENTIAL thresholds (same domain as runtime readings)
    send_bluetooth_message("Phase 4: Calculating differential wall thresholds\r\n");


    for (int i=0;i<4;i++){
        float frac   = (i < 2) ? 0.40f : 0.30f;   // front vs side
        float margin = 3.0f * sensor_cal.noise_levels[i];
        float th     = frac * (float)diff_avg[i] + margin;

        if (th < 60.0f)   th = 60.0f;             // reasonable floor
        if (th > 3500.0f) th = 3500.0f;

        sensor_cal.wall_thresholds[i] = (uint16_t)(th + 0.5f);
    }

    send_bluetooth_message("Differential wall thresholds:\r\n");
    send_bluetooth_printf("  Front Left:  %u\r\n", sensor_cal.wall_thresholds[0]);
    send_bluetooth_printf("  Front Right: %u\r\n", sensor_cal.wall_thresholds[1]);
    send_bluetooth_printf("  Side Left:   %u\r\n", sensor_cal.wall_thresholds[2]);
    send_bluetooth_printf("  Side Right:  %u\r\n", sensor_cal.wall_thresholds[3]);



    // Phase 5: Battery Baseline Measurement
    send_bluetooth_message("Phase 5: Battery voltage baseline measurement\r\n");

    uint32_t battery_sum = 0;
    for (int i = 0; i < 20; i++) {
        battery_sum += read_adc_channel(ADC_CHANNEL_0);
        HAL_Delay(10);
    }
    sensor_cal.battery_baseline = battery_sum / 20;

    float battery_voltage = (sensor_cal.battery_baseline * 3.3f) / 4096.0f;
    send_bluetooth_printf("Battery baseline: %d (%.2fV)\r\n",
                         sensor_cal.battery_baseline, battery_voltage);

    // Phase 6: Sensor Health Validation
//    send_bluetooth_message("Phase 6: Sensor health validation\r\n");
//
    bool all_sensors_healthy = true;
//
//    // Check each sensor
//    const char* sensor_names[4] = {"Front Left", "Front Right", "Side Left", "Side Right"};
//
//    for (int i = 0; i < 4; i++) {
//        bool sensor_healthy = true;
//
//        // Check if sensor shows reasonable differential response
//        if (differential[i] < 50) {
//            send_bluetooth_printf("⚠️ %s: Low differential response (%d)\r\n",
//                                 sensor_names[i], differential[i]);
//            sensor_healthy = false;
//        }
//
//        // Check noise levels
//        if (sensor_cal.noise_levels[i] > 200) {
//            send_bluetooth_printf("⚠️ %s: High noise level (%.1f)\r\n",
//                                 sensor_names[i], sensor_cal.noise_levels[i]);
//            sensor_healthy = false;
//        }
//
//        // Check if readings are within reasonable ADC range
//        if (sensor_cal.ambient_baseline[i] > 3800 || sensor_cal.ambient_baseline[i] < 10) {
//            send_bluetooth_printf("⚠️ %s: Ambient reading out of range (%d)\r\n",
//                                 sensor_names[i], sensor_cal.ambient_baseline[i]);
//            sensor_healthy = false;
//        }
//
//        if (sensor_healthy) {
//            send_bluetooth_printf("✅ %s: Healthy\r\n", sensor_names[i]);
//        } else {
//            all_sensors_healthy = false;
//        }
//    }

    // Check battery
    if (battery_voltage < 3.0f) {
        send_bluetooth_message("⚠️ Battery: Low voltage detected\r\n");
        all_sensors_healthy = false;
    } else if (battery_voltage > 4.5f) {
        send_bluetooth_message("⚠️ Battery: Voltage too high\r\n");
        all_sensors_healthy = false;
    } else {
        send_bluetooth_message("✅ Battery: Healthy\r\n");
    }

    // Phase 7: Calibration Complete
    sensor_cal.calibration_valid = all_sensors_healthy;

    turn_off_emitters(); // Save power

    if (sensor_cal.calibration_valid) {
        send_bluetooth_message("✅ SENSOR CALIBRATION COMPLETE - All systems nominal\r\n");
        send_bluetooth_message("Dynamic thresholds will be used for wall detection\r\n");
    } else {
        send_bluetooth_message("⚠️ SENSOR CALIBRATION COMPLETE - Some issues detected\r\n");
        send_bluetooth_message("Robot will continue with degraded sensor performance\r\n");
    }

    send_bluetooth_message("=====================================\r\n");
}

/**
 * @brief Get calibrated wall threshold for specific sensor
 * @param sensor_index: 0=Front_Left, 1=Front_Right, 2=Side_Left, 3=Side_Right
 * @return Calibrated threshold value
 */
uint16_t get_calibrated_threshold(int sensor_index)
{
    if (sensor_index < 0 || sensor_index > 3 || !sensor_cal.calibration_valid) {
        // Return default thresholds if calibration failed
        return (sensor_index < 2) ? WALL_THRESHOLD_FRONT : WALL_THRESHOLD_SIDE;
    }

    return sensor_cal.wall_thresholds[sensor_index];
}

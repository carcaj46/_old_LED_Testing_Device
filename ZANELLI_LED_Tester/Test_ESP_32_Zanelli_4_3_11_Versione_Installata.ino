/*
 * Release 4.3.11
 * Modifiche:
 * - Il PWM effettivo è forzato tra 20% e 99%.
 * - Il valore visualizzato sul display va da 10% (quando effettivo 20%) a 100% (quando effettivo 99%).
 * - Mappatura lineare: display_percent = 10 + ((last_pwm_percent - 20)*90/79)
 */

#include <Arduino.h>
#include <driver/ledc.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>

#define START_BUTTON_PIN GPIO_NUM_2
#define RESET_BUTTON_PIN GPIO_NUM_1
#define FORWARD_BUTTON_PIN GPIO_NUM_3
#define BACK_BUTTON_PIN GPIO_NUM_4
#define STATUS_BUTTON_PIN GPIO_NUM_12
#define STOPLED_PIN GPIO_NUM_45
#define START_LED_PIN GPIO_NUM_46

#define SENSOR_1_PIN GPIO_NUM_5
#define SENSOR_2_PIN GPIO_NUM_6
#define SENSOR_3_PIN GPIO_NUM_7
#define LIMIT_SENSOR_PIN GPIO_NUM_10

#define MOTOR_DH GPIO_NUM_36
#define MOTOR_DL GPIO_NUM_35
#define MOTOR_EH GPIO_NUM_38
#define MOTOR_EL GPIO_NUM_37

#define ADC_PIN GPIO_NUM_12

#define LEDC_TIMER_12_BIT 12
#define LEDC_BASE_FREQ 9000
int PWM_D = 250; 
int PWM_E = 250; 
int SLOW_PWM = 50; 
int MID_PWM = 150; 

void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
    uint32_t duty = (4095 / valueMax) * min(value, valueMax);
    ledcWrite(channel, duty);
}

#define SLOW_PIN GPIO_NUM_21
#define I2C_SDA_PIN GPIO_NUM_8
#define I2C_SCL_PIN GPIO_NUM_9

#define EEPROM_ADDR_TOTAL_CYCLES 0
#define EEPROM_ADDR_COMPLETED_CYCLES 4
#define EEPROM_ADDR_RUN_STATE 8

LiquidCrystal_I2C lcd(0x27, 16, 2);

bool home = true;
bool input_on = false;
bool ready = false;
bool manual_on = false;
bool go_to_ready = false;
bool go_to_0 = false;
bool is_searching_0 = false;
bool limit_error = false;
bool limitSensorPreviouslyLow = false;
bool run_on = false;
bool motor_direction_fw = true;
bool ledState = false;

unsigned long total_cycles = 0;
unsigned long completed_cycles = 0;

volatile bool status_flag = false;
volatile bool stop_flag = false;
volatile bool forwardButtonPressed = false;
volatile bool backButtonPressed = false;

bool slow_mode_active = false;
bool mid_mode_active = false;

unsigned long lastBlinkTime = 0;
bool blinkState = true;

struct SystemState {
    bool home;
    bool run_on;
    bool input_on;
    bool ready;
    bool manual_on;
    bool go_to_ready;
    bool go_to_0;
    bool is_searching_0;
    int input_digit;
    unsigned long last_button_press_time;
    bool motor_direction_fw;
};

SystemState previousState;

int input_digit = 0;
unsigned long last_button_press_time = 0;
volatile unsigned long reset_button_press_time = 0;
volatile bool reset_in_progress = false;
bool just_entered_input_mode = false;

volatile bool sensor1_flag = false;
volatile bool sensor2_flag = false;
volatile bool sensor3_flag = false;
volatile bool limitSensor_flag = false;

bool sensor1_prev_state = HIGH;
bool sensor2_prev_state = HIGH;
bool sensor3_prev_state = HIGH;

OneButton startButton(START_BUTTON_PIN, true);
OneButton resetButton(RESET_BUTTON_PIN, true);
OneButton statusButton(STATUS_BUTTON_PIN, true);
OneButton forwardButton(FORWARD_BUTTON_PIN, true);
OneButton backButton(BACK_BUTTON_PIN, true);

float last_pwm_percent = 0.0;
float lastDisplayedPWM = -1.0; // ultimo PWM mostrato

void updatePWMFromADC() {
    int adcValue = analogRead(ADC_PIN);
    float maxADCAt3V = (3.0/3.3)*4095.0; 
    float pwm_percent = ((float)adcValue / maxADCAt3V) * 99.0;
    if (pwm_percent > 99.0) pwm_percent = 99.0;
    if (pwm_percent < 20.0) pwm_percent = 20.0; // Forziamo il minimo a 20%
    // Il massimo resta 99%

    PWM_D = (int)(255.0 * (pwm_percent / 100.0));
    PWM_E = PWM_D;
    last_pwm_percent = pwm_percent;
}

void activateSlowMode() {
    slow_mode_active = true;
    mid_mode_active = false;
}

void deactivateSlowMode() {
    slow_mode_active = false;
}

void activateMidMode() {
    mid_mode_active = true;
    slow_mode_active = false;
}

void deactivateMidMode() {
    mid_mode_active = false;
}

void moveMotorForward(const char* reason) {
    // Rimuoviamo la lettura ADC qui. PWM_D/PWM_E restano come sono.
    if (mid_mode_active) {
        PWM_D = MID_PWM;
        PWM_E = MID_PWM;
    } else if (slow_mode_active) {
        PWM_D = SLOW_PWM;
        PWM_E = SLOW_PWM;
    }

    motor_direction_fw = true;
    ledcAnalogWrite(MOTOR_DH, PWM_D);
    digitalWrite(MOTOR_DL, HIGH);
    digitalWrite(MOTOR_EL, LOW);
    delay(10);
}

void moveMotorBackward(const char* reason) {
    if (mid_mode_active) {
        PWM_D = MID_PWM;
        PWM_E = MID_PWM;
    } else if (slow_mode_active) {
        PWM_D = SLOW_PWM;
        PWM_E = SLOW_PWM;
    }

    motor_direction_fw = false;
    ledcAnalogWrite(MOTOR_EH, PWM_E);
    digitalWrite(MOTOR_DL, LOW);
    digitalWrite(MOTOR_EL, HIGH);
}

int mapPwmToDisplay(float actual_pwm) {
    // Mappatura lineare:
    // actual: 20% -> display:10%
    // actual: 99% -> display:100%
    // display = 10 + ((actual_pwm -20)*90/79)
    return (int)round(10.0 + ((actual_pwm - 20.0)*90.0/79.0));
}

void updatePwmOnDisplay() {
    int display_val = mapPwmToDisplay(last_pwm_percent);
    float disp_f = (float)display_val;
    if (disp_f != lastDisplayedPWM) {
        lcd.setCursor(0,1);
        char pwm_str[5];
        snprintf(pwm_str,5,"%3d%%",display_val);
        lcd.print(pwm_str);
        //for (int i = 4; i < 16; i++) {
        //    lcd.print(' ');
        //}
        lastDisplayedPWM = disp_f;
    }
}

void setup() {
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
    pinMode(LIMIT_SENSOR_PIN, INPUT_PULLUP);
    pinMode(FORWARD_BUTTON_PIN, INPUT_PULLUP);
    pinMode(BACK_BUTTON_PIN, INPUT_PULLUP);

    pinMode(STOPLED_PIN, OUTPUT);
    pinMode(START_LED_PIN, OUTPUT);
    digitalWrite(START_LED_PIN, LOW);
    digitalWrite(STOPLED_PIN, LOW);

    pinMode(SENSOR_1_PIN, INPUT_PULLUP);
    pinMode(SENSOR_2_PIN, INPUT_PULLUP);
    pinMode(SENSOR_3_PIN, INPUT_PULLUP);
    pinMode(LIMIT_SENSOR_PIN, INPUT_PULLUP);

    pinMode(MOTOR_DH, OUTPUT);
    pinMode(MOTOR_DL, OUTPUT);
    pinMode(MOTOR_EH, OUTPUT);
    pinMode(MOTOR_EL, OUTPUT);

    ledcAttach(MOTOR_DH, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);
    ledcAttach(MOTOR_EH, LEDC_BASE_FREQ, LEDC_TIMER_12_BIT);

    digitalWrite(MOTOR_DL, HIGH);
    digitalWrite(MOTOR_EL, HIGH);
    
    ledcAnalogWrite(MOTOR_DH, 0);  
    ledcAnalogWrite(MOTOR_EH, 0);  
    digitalWrite(MOTOR_DL, HIGH);  
    digitalWrite(MOTOR_EL, HIGH);  
    stopMotor();

    pinMode(SLOW_PIN, OUTPUT);
    digitalWrite(SLOW_PIN, HIGH);

    startButton.attachClick([](){onStartPress();});
    startButton.attachLongPressStart([](){onStartLongPress();});

    resetButton.attachClick([](){onResetPress();});
    resetButton.attachLongPressStart([](){onResetLongPress();});

    statusButton.attachClick([](){
        // Nessuna stampa
    });

    forwardButton.attachClick([]() {
        if (limit_error) {
            limit_error = false;
            resetToHomeState();
        } else {
            forwardButtonPressed = true;
        }
    });

    backButton.attachClick([]() {
        if (limit_error) {
            limit_error = false;
            resetToHomeState();
        } else {
            backButtonPressed = true;
        }
    });

    EEPROM.begin(512);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    lcd.init();
    lcd.backlight();

    loadSystemState();
    updateDisplay();

    pinMode(ADC_PIN, INPUT);
}

void loop() {
    updatePWMFromADC(); // Lettura continua dell'ADC e clamping a 20%-99%

    if (limit_error) {
        return;
    }

    startButton.tick();
    resetButton.tick();
    statusButton.tick();
    forwardButton.tick();
    backButton.tick();

    handleSensorPolling();

    if (input_on) {
        handleInputBlink();
    }

    if (input_on) {
        if (forwardButtonPressed) {
            handleForwardButtonPress();
            forwardButtonPressed = false;
        }
        if (backButtonPressed) {
            handleBackButtonPress();
            backButtonPressed = false;
        }
    }

    if (is_searching_0) {
        if (digitalRead(LIMIT_SENSOR_PIN) == LOW) {
            stopRunMode();
            is_searching_0 = false;
            limit_error = true;
            handleLimitError();
            updateDisplay();
        } else if ((digitalRead(SENSOR_1_PIN) == LOW) && (digitalRead(SENSOR_2_PIN) == LOW)) {
            stopMotor();
            deactivateSlowMode();
            deactivateMidMode();
            is_searching_0 = false;
            ready = true;
            go_to_ready = true;
            updateDisplay();
        }
    }

    if (run_on) {
        if (digitalRead(LIMIT_SENSOR_PIN) == LOW) {
            stopRunMode();
            limit_error = true;
            handleLimitError();
            updateDisplay();
            return;
        }

        if (motor_direction_fw && (digitalRead(SENSOR_3_PIN) == LOW)) {
            stopMotor();
            moveMotorBackward("Run Mode cambio direzione (sensore 3)");
        } else if (!motor_direction_fw && (digitalRead(SENSOR_1_PIN) == LOW && digitalRead(SENSOR_2_PIN) == LOW)) {
            stopMotor();
            completed_cycles++;
            saveCycleCount();
            updateDisplay();

            if (completed_cycles >= total_cycles && total_cycles > 0) {
                run_on = false;
                digitalWrite(STOPLED_PIN, HIGH);
                showJobEnded();
            } else {
                moveMotorForward("Run Mode nuovo ciclo");
            }
        }
    }

    if ((home || manual_on || go_to_0) && !run_on && !input_on && !ready && !go_to_ready && !is_searching_0 && !stop_flag) {
        if (digitalRead(FORWARD_BUTTON_PIN) == LOW) {
            activateSlowMode();       
            moveMotorForward("Manual Forward (Slow)");

            unsigned long pressStart = millis();
            while (digitalRead(FORWARD_BUTTON_PIN) == LOW) {
                if (!mid_mode_active && (millis() - pressStart > 3000)) {
                    activateMidMode();
                    ledcAnalogWrite(MOTOR_DH, MID_PWM);
                }

                if (digitalRead(LIMIT_SENSOR_PIN) == LOW) {
                    stopMotor();
                    limit_error = true;
                    handleLimitError();
                    updateDisplay();
                    deactivateSlowMode(); 
                    deactivateMidMode();
                    return;
                }
                delay(100);
            }

            stopMotor();
            deactivateSlowMode();
            deactivateMidMode();
        } 
        else if (digitalRead(BACK_BUTTON_PIN) == LOW) {
            activateSlowMode();        
            moveMotorBackward("Manual Back (Slow)");

            unsigned long pressStart = millis();
            while (digitalRead(BACK_BUTTON_PIN) == LOW) {
                if (!mid_mode_active && (millis() - pressStart > 3000)) {
                    activateMidMode();
                    ledcAnalogWrite(MOTOR_EH, MID_PWM);
                }

                if (digitalRead(LIMIT_SENSOR_PIN) == LOW) {
                    stopMotor();
                    limit_error = true;
                    handleLimitError();
                    updateDisplay();
                    deactivateSlowMode();
                    deactivateMidMode();
                    return;
                }
                delay(100);
            }

            stopMotor();
            deactivateSlowMode();
            deactivateMidMode();
        }
    }

    if (home && total_cycles == 0 && !run_on) {
        digitalWrite(START_LED_PIN, HIGH);
    } else if (home && total_cycles > 0 && completed_cycles < total_cycles && !run_on) {
        static unsigned long lastLedToggleTime = 0;
        if (millis() - lastLedToggleTime >= 1000) {
            lastLedToggleTime = millis();
            ledState = !ledState;
            digitalWrite(START_LED_PIN, ledState);
        }
    } else if ((go_to_ready || go_to_0) && !run_on) {
        static unsigned long lastLedToggleTime = 0;
        if (millis() - lastLedToggleTime >= 300) {
            lastLedToggleTime = millis();
            ledState = !ledState;
            digitalWrite(START_LED_PIN, ledState);
        }
    } else if (home && total_cycles > 0 && completed_cycles >= total_cycles && !run_on) {
        digitalWrite(START_LED_PIN, LOW);
    }

    // Aggiorniamo il PWM sul display se cambia
    updatePwmOnDisplay();
}

void handleSensorPolling() {
    bool sensor1_current_state = digitalRead(SENSOR_1_PIN);
    bool sensor2_current_state = digitalRead(SENSOR_2_PIN);
    bool sensor3_current_state = digitalRead(SENSOR_3_PIN);

    if (sensor1_current_state != sensor1_prev_state) {
        if (sensor1_prev_state == HIGH && sensor1_current_state == LOW) {
            handleSensor1Event();
        }
        sensor1_prev_state = sensor1_current_state;
    }

    if (sensor2_current_state != sensor2_prev_state) {
        if (sensor2_prev_state == HIGH && sensor2_current_state == LOW) {
            handleSensor2Event();
        }
        sensor2_prev_state = sensor2_current_state;
    }

    if (sensor3_current_state != sensor3_prev_state) {
        if (sensor3_prev_state == HIGH && sensor3_current_state == LOW) {
            handleSensor3Event();
        }
        sensor3_prev_state = sensor3_current_state;
    }

    bool currentLimitSensorState = digitalRead(LIMIT_SENSOR_PIN);
    if (currentLimitSensorState == LOW && !limitSensorPreviouslyLow) {
        handleLimitSensorEvent();
        limitSensorPreviouslyLow = true;
    } else if (currentLimitSensorState == HIGH) {
        limitSensorPreviouslyLow = false;
    }
}

void onStartPress() {
    dispatchStartButtonAction();
}

void onStartLongPress() {
    if (input_on) {
        handleInputModeStartLongPress();
    } else if (is_searching_0) {
        resetToHomeState();
        updateDisplay();
    } else if (run_on) {
        stopRunMode();
        ready = false;
        updateDisplay();
    } else if (home && !run_on && !input_on && !is_searching_0 && !stop_flag && (go_to_ready || go_to_0)) {
        handleGoToModeLongPress();
    } else if (home && !run_on && !input_on && !is_searching_0 && !stop_flag) {
        handleHomeModeStartLongPress();
    }
}

void onResetPress() {
    if (stop_flag) {
        stop_flag = false;
        restorePreviousState();
    } else {
        stop_flag = true;
        storeCurrentState();

        home = false;
        run_on = false;
        input_on = false;
        ready = false;
        manual_on = false;
        go_to_ready = false;
        go_to_0 = false;
        is_searching_0 = false;
    }
    stopMotor();
    updateDisplay();
}

void onResetLongPress() {
    performReset();
    updateDisplay();
}

void handleForwardButtonPress() {
    if (input_on) {
        int increment_value = 0;
        switch (input_digit) {
            case 1: increment_value = 1; break;
            case 2: increment_value = 10; break;
            case 3: increment_value = 100; break;
            case 4: increment_value = 1000; break;
            case 5: increment_value = 10000; break;
        }

        total_cycles += increment_value;
        updateDisplay();
    }
}

void handleBackButtonPress() {
    if (input_on) {
        int decrement_value = 0;
        switch (input_digit) {
            case 1: decrement_value = 1; break;
            case 2: decrement_value = 10; break;
            case 3: decrement_value = 100; break;
            case 4: decrement_value = 1000; break;
            case 5: decrement_value = 10000; break;
        }

        if (total_cycles >= decrement_value) {
            total_cycles -= decrement_value;
        } else {
            total_cycles = 0;
        }
        updateDisplay();
    }
}

void dispatchStartButtonAction() {
    if (input_on && !run_on && !home) {
        handleInputModeStartPress();
    } else if (home && !run_on && !input_on && go_to_ready) {
        startRunningCycle();
    } else if (home && !run_on && !input_on && go_to_0) {
        startSearch0Cycle();
    } else if (home && !run_on && !input_on) {
        handleHomeModeStartPress();
    }
}

void handleHomeModeStartPress() {
    if (home && !run_on && !input_on && !ready && !go_to_ready && !go_to_0 && !is_searching_0 && !stop_flag) {
        input_on = true;
        home = false;
        input_digit = 1;
        updateDisplay();
        updateStartLED();
    }
}

void handleInputModeStartPress() {
    input_digit++;
    if (input_digit > 5) {
        input_digit = 1;
    }
    updateDisplay();
}

void handleHomeModeStartLongPress() {
    if (total_cycles > 0 && completed_cycles < total_cycles && home && !run_on && !input_on && !is_searching_0 && !stop_flag) {
        if ((digitalRead(SENSOR_1_PIN) == LOW) && (digitalRead(SENSOR_2_PIN) == LOW)) {
            ready = true;
            go_to_ready = true;
        } else {
            go_to_0 = true;
        }
    }
    updateDisplay();
}

void handleInputModeStartLongPress() {
    input_on = false;
    home = true;
    run_on = false;
    ready = false;
    manual_on = false;
    go_to_ready = false;
    go_to_0 = false;
    is_searching_0 = false;
    stop_flag = false;

    saveCycleCount();
    updateDisplay();
}

void handleGoToModeLongPress() {
    resetToHomeState();
    updateDisplay();
}

void resetToHomeState() {
    if (limit_error) {
        return;
    }
    home = true;
    run_on = false;
    input_on = false;
    ready = false;
    manual_on = false;
    go_to_ready = false;
    go_to_0 = false;
    is_searching_0 = false;
}

void performReset() {
    digitalWrite(STOPLED_PIN, LOW);
    total_cycles = 0;
    completed_cycles = 0;
    home = true;
    run_on = false;
    input_on = false;
    ready = false;
    manual_on = false;
    go_to_ready = false;
    go_to_0 = false;
    is_searching_0 = false;
    stop_flag = false;
    saveCycleCount();
}

void saveCycleCount() {
    EEPROM.write(EEPROM_ADDR_TOTAL_CYCLES, total_cycles & 0xFF);
    EEPROM.write(EEPROM_ADDR_TOTAL_CYCLES + 1, (total_cycles >> 8) & 0xFF);
    EEPROM.write(EEPROM_ADDR_TOTAL_CYCLES + 2, (total_cycles >> 16) & 0xFF);
    EEPROM.write(EEPROM_ADDR_TOTAL_CYCLES + 3, (total_cycles >> 24) & 0xFF);

    EEPROM.write(EEPROM_ADDR_COMPLETED_CYCLES, completed_cycles & 0xFF);
    EEPROM.write(EEPROM_ADDR_COMPLETED_CYCLES + 1, (completed_cycles >> 8) & 0xFF);
    EEPROM.write(EEPROM_ADDR_COMPLETED_CYCLES + 2, (completed_cycles >> 16) & 0xFF);
    EEPROM.write(EEPROM_ADDR_COMPLETED_CYCLES + 3, (completed_cycles >> 24) & 0xFF);

    EEPROM.commit();
}

void loadSystemState() {
    total_cycles = EEPROM.read(EEPROM_ADDR_TOTAL_CYCLES) |
                   (EEPROM.read(EEPROM_ADDR_TOTAL_CYCLES + 1) << 8) |
                   (EEPROM.read(EEPROM_ADDR_TOTAL_CYCLES + 2) << 16) |
                   (EEPROM.read(EEPROM_ADDR_TOTAL_CYCLES + 3) << 24);

    completed_cycles = EEPROM.read(EEPROM_ADDR_COMPLETED_CYCLES) |
                       (EEPROM.read(EEPROM_ADDR_COMPLETED_CYCLES + 1) << 8) |
                       (EEPROM.read(EEPROM_ADDR_COMPLETED_CYCLES + 2) << 16) |
                       (EEPROM.read(EEPROM_ADDR_COMPLETED_CYCLES + 3) << 24);
}

void storeCurrentState() {
    previousState.home = home;
    previousState.run_on = run_on;
    previousState.input_on = input_on;
    previousState.ready = ready;
    previousState.manual_on = manual_on;
    previousState.go_to_ready = go_to_ready;
    previousState.go_to_0 = go_to_0;
    previousState.is_searching_0 = is_searching_0;
    previousState.input_digit = input_digit;
    previousState.last_button_press_time = last_button_press_time;
    previousState.motor_direction_fw = motor_direction_fw;
}

void restorePreviousState() {
    home = previousState.home;
    run_on = previousState.run_on;
    input_on = previousState.input_on;
    ready = previousState.ready;
    manual_on = previousState.manual_on;
    go_to_ready = previousState.go_to_ready;
    go_to_0 = previousState.go_to_0;
    is_searching_0 = previousState.is_searching_0;
    input_digit = previousState.input_digit;
    last_button_press_time = previousState.last_button_press_time;
    motor_direction_fw = previousState.motor_direction_fw;

    if (run_on) {
        if (motor_direction_fw) {
            moveMotorForward("Restore Previous State");
        } else {
            moveMotorBackward("Restore Previous State");
        }

        delay(100);

        bool sensor1State = digitalRead(SENSOR_1_PIN);
        bool sensor2State = digitalRead(SENSOR_2_PIN);
        bool sensor3State = digitalRead(SENSOR_3_PIN);

        if (motor_direction_fw && sensor3State == LOW) {
            stopMotor();
            moveMotorBackward("Sensor Condition");
        } else if (!motor_direction_fw && (sensor1State == LOW && sensor2State == LOW)) {
            stopMotor();
            completed_cycles++;
            saveCycleCount();
            updateDisplay();

            if (completed_cycles >= total_cycles) {
                run_on = false;
                updateDisplay();
            } else {
                moveMotorForward("Sensor Condition");
            }
        }
    }
}

void updateDisplay() {
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("                ");

    if (is_searching_0) {
        lcd.setCursor(0, 0);
        lcd.print("Searching 0");
    } else if (run_on) {
        lcd.setCursor(0, 0);
        lcd.print("Running");
        char total_cycles_str[6];
        sprintf(total_cycles_str, "%05lu", total_cycles);
        lcd.setCursor(11, 0);
        lcd.print(total_cycles_str);

        lcd.setCursor(5, 1);
        lcd.print("Comp.");
        char completed_cycles_str[6];
        sprintf(completed_cycles_str, "%05lu", completed_cycles);
        lcd.setCursor(11, 1);
        lcd.print(completed_cycles_str);
    } else if (go_to_ready) {
        lcd.setCursor(0, 0);
        lcd.print("Press Start");
        char total_cycles_str[6];
        sprintf(total_cycles_str, "%05lu", total_cycles);
        lcd.setCursor(11, 0);
        lcd.print(total_cycles_str);

        lcd.setCursor(5, 1);
        lcd.print("to Run");
        char completed_cycles_str[6];
        sprintf(completed_cycles_str, "%05lu", completed_cycles);
        lcd.setCursor(11, 1);
        lcd.print(completed_cycles_str);
    } else if (go_to_0) {
        lcd.setCursor(0, 0);
        lcd.print("Press Start to");
        lcd.setCursor(5, 1);
        lcd.print("Goto0");
    } else if (home && !run_on && !input_on && !manual_on && !go_to_ready && !go_to_0 && !is_searching_0 && !stop_flag) {
        lcd.setCursor(0, 0);
        lcd.print("Helitas");
        if (total_cycles > 0) {
            char buffer[6];
            sprintf(buffer, "%05lu", total_cycles);
            lcd.setCursor(11, 0);
            lcd.print(buffer);

            lcd.setCursor(5, 1);
            lcd.print("Home");
            char completed_cycles_str[6];
            sprintf(completed_cycles_str, "%05lu", completed_cycles);
            lcd.setCursor(11, 1);
            lcd.print(completed_cycles_str);
        }
    } else if (stop_flag) {
        lcd.setCursor(0, 0);
        lcd.print("STOP Attivo");
    } else if (input_on) {
        lcd.setCursor(0, 0);
        lcd.print("Set");
        char buffer[6];
        sprintf(buffer, "%05lu", total_cycles);
        lcd.setCursor(11, 0);
        for (int i = 0; i < 5; i++) {
            if (i == (5 - input_digit)) {
                if (blinkState) {
                    lcd.print(' ');
                } else {
                    lcd.print(buffer[i]);
                }
            } else {
                lcd.print(buffer[i]);
            }
        }

        lcd.setCursor(0, 1);
        lcd.print("Comp");
        char completed_cycles_str[6];
        sprintf(completed_cycles_str, "%05lu", completed_cycles);
        lcd.setCursor(11, 1);
        lcd.print(completed_cycles_str);
    } else if (manual_on) {
        lcd.setCursor(0, 0);
        lcd.print("Manual Mode");
    } else {
        lcd.setCursor(0, 0);
        lcd.print("Default State");
    }

    // Stampa iniziale PWM dopo updateDisplay() con la nuova mappatura
    int display_val = mapPwmToDisplay(last_pwm_percent);
    lcd.setCursor(0,1);
    char pwm_str[5];
    snprintf(pwm_str,5,"%3d%%",display_val);
    lcd.print(pwm_str);
    //for (int i = 4; i < 16; i++) {
    //    lcd.print(' ');
    //}
    lastDisplayedPWM = (float)display_val;
}

void handleInputBlink() {
    if (millis() - lastBlinkTime >= 500) {
        lastBlinkTime = millis();
        blinkState = !blinkState;
        updateDisplay();
    }
}

void handleSensor1Event() {
}

void handleSensor2Event() {
}

void handleSensor3Event() {
}

void handleLimitSensorEvent() {
}

void startSearch0Cycle() {
    if ((digitalRead(SENSOR_1_PIN) == LOW) && (digitalRead(SENSOR_2_PIN) == LOW)) {
        is_searching_0 = false;
        go_to_0 = false;
        ready = true;
        go_to_ready = true;
        updateDisplay();
    } else {
        activateSlowMode();
        moveMotorBackward("Ricerca 0");
        is_searching_0 = true;
        go_to_0 = false;
        updateDisplay();
    }
}

void startRunningCycle() {
    run_on = true;
    motor_direction_fw = true;
    go_to_ready = false;
    moveMotorForward("Run Mode start");
    updateDisplay();
}

void enterErrorMode(bool isLimitError) {
    home = false;
    run_on = false;
    input_on = false;
    ready = false;
    manual_on = false;
    go_to_ready = false;
    go_to_0 = false;
    is_searching_0 = false;
    stop_flag = true;

    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("                ");
    if (isLimitError) {
        lcd.setCursor(0, 0);
        lcd.print("ERRORE: LIMITE");
        lcd.setCursor(0, 1);
        lcd.print("Block");
    } else {
        lcd.setCursor(0, 0);
        lcd.print("ERRORE: GENERALE");
    }
}

void stopRunMode() {
    stopMotor();
    run_on = false;
    ready = false;
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("STOP-RUN Attivo");
}

void enterLimitErrorMode() {
    limit_error = true;
    handleLimitError();
}

void handleLimitError() {
    home = false;
    run_on = false;
    input_on = false;
    ready = false;
    manual_on = false;
    go_to_ready = false;
    go_to_0 = false;
    is_searching_0 = false;
    limit_error = true;
    stopMotor();
    delay(500);
    if (motor_direction_fw) {
        moveMotorBackward("Limit Error");
    } else {
        moveMotorForward("Limit Error");
    }
    delay(300);
    stopMotor();
    manual_on = true;
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("Limit Error");
    lcd.setCursor(5, 1);
    lcd.print("P. F/B");
    while (limit_error) {
        resetButton.tick();
        if (resetButton.isLongPressed()) {
            limit_error = false;
            onResetLongPress();
            return;
        }
        if (digitalRead(FORWARD_BUTTON_PIN) == LOW) {
            limit_error = false;
            resetToHomeState();
            return;
        }
        if (digitalRead(BACK_BUTTON_PIN) == LOW) {
            limit_error = false;
            resetToHomeState();
            return;
        }
        delay(100);
    }
}

void showJobEnded() {
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("Cycles");
    char total_cycles_str[6];
    sprintf(total_cycles_str, "%05lu", total_cycles);
    lcd.setCursor(11, 0);
    lcd.print(total_cycles_str);

    lcd.setCursor(5, 1);
    lcd.print("Exec.");
    char completed_cycles_str[6];
    sprintf(completed_cycles_str, "%05lu", completed_cycles);
    lcd.setCursor(11, 1);
    lcd.print(completed_cycles_str);
}

void updateStartLED() {
    if (input_on) {
        static unsigned long lastToggleTime = 0;
        unsigned long currentTime = millis();
        if (currentTime - lastToggleTime >= 500) {
            ledState = !ledState;
            digitalWrite(START_LED_PIN, ledState);
            lastToggleTime = currentTime;
        }
    } else {
        digitalWrite(START_LED_PIN, LOW);
    }
}

void stopMotor() {
    ledcAnalogWrite(MOTOR_DH, 0);
    ledcAnalogWrite(MOTOR_EH, 0);
}
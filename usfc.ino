/*
 * Copyright (c) 2017-2018, Theodor-Iulian Ciobanu
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

#define VERSION "0.3.0"

// error codes
#define SENSOR_NOTFOUND  0xD
#define SENSOR_READERROR 0xB

// data wire is plugged into port 4 on the Arduino
#define ONE_WIRE_BUS 4
// PWM pin to fan
#define PWM_PIN 3
// RPM pin from fan
#define RPM_PIN 2

// polling interval
#define MIN_POLL_INTERVAL 0
#define MAX_POLL_INTERVAL 3600000
unsigned long poll_ms = 1000;

// temperature limits
#define MIN_START_TEMP 0
#define MAX_START_TEMP 64
#define MIN_MAX_TEMP   0
#define MAX_MAX_TEMP   128
// temperatures to start fan and for max speed
char start_temp = 32;
char maxfan_temp = 52;
char delta_temp = maxfan_temp - start_temp;

// cooling mode: 0 - quiet (logarithmic), 1 - default (linear), 2 - performance (exponential)
unsigned char mode = 1;
float log_b = log(255 / 0.4) / (float)delta_temp;
float log_a = (float)255 / exp(log_b * maxfan_temp);
float exp_a = (255 - 0.4) / log((float)maxfan_temp / start_temp);
float exp_b = exp(255 / exp_a) / (float)maxfan_temp;

// setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS);
// pass oneWire reference to DallasTemperature
DallasTemperature sensors(&oneWire);
// sensor address
DeviceAddress sensor;

#define MAX_INPUT_SIZE 47
// a string to hold incoming data
char input_string[MAX_INPUT_SIZE + 1];
// whether the string is complete
boolean string_complete = false;
// current string length
unsigned char string_pos = 0;

// number of rotations in time interval
unsigned long rotations;
// fan divider: 2 - unipole hall effect sensor, 8 - bipole hall effect sensor
#define MIN_FAN_DIV 1
#define MAX_FAN_DIV 16384
unsigned char fan_div = 2;

// monitor mode: 0 - disabled, 1 - one time, 2 - continous
unsigned char stats = 2;

// echo serial input back
unsigned char serial_echo = 0;

// number of failed reads
#define MIN_MAX_ERRORS 1
#define MAX_MAX_ERRORS 3600
unsigned char err_count = 0;
unsigned char max_errors = 16;

void count_rotations() {
    rotations++;
}

void error_blink(unsigned int errnum) {
    while (1) {
        unsigned int code = errnum;

        // error code
        do {
            digitalWrite(LED_BUILTIN, code & 1);
            delay(500);
            code >>= 1;
        } while (code);

        // pause
        digitalWrite(LED_BUILTIN, LOW);
        delay(2000);
    }

    exit(-errnum);
}

void show_info() {
    Serial.print(F("{\"version\": \""
        VERSION
        "\", \"mode\": "));
    Serial.print(mode, DEC);
    Serial.print(F(", \"start\": "));
    Serial.print(start_temp, DEC);
    Serial.print(F(", \"threshold\": "));
    Serial.print(maxfan_temp, DEC);
    Serial.print(F(", \"polling\": "));
    Serial.print(poll_ms, DEC);
    Serial.print(F(", \"errlimit\": "));
    Serial.print(max_errors, DEC);
    Serial.print(F(", \"divider\": "));
    Serial.print(fan_div, DEC);
    Serial.print(F(", \"stats\": "));
    Serial.print(stats, DEC);
    Serial.print(F(", \"echo\": "));
    Serial.print(serial_echo, DEC);
    Serial.print(F(", \"uptime\": "));
    Serial.print(millis(), DEC);
    Serial.println("}");
}

void show_help() {
    Serial.print(F("{\"version\": \""
        VERSION
        "\", \"mode\": {\"help\": \"cooling mode\""
        ", \"values\": {\"0\": \"quiet (logarithmic)\""
        ", \"1\": \"default (linear)\""
        ", \"2\": \"performance (exponential)\"}, \"current\": "));
    Serial.print(mode, DEC);
    Serial.print(F("}, \"start\": {\"help\": \"temperature to start fan\""
        ", \"range\": {\"min\": "));
    Serial.print(MIN_START_TEMP, DEC);
    Serial.print(F(", \"max\": "));
    Serial.print(MAX_START_TEMP, DEC);
    Serial.print(F("}, \"current\": "));
    Serial.print(start_temp, DEC);
    Serial.print(F("}, \"threshold\": {\"help\": \"temperature to go full speed\""
        ", \"range\": {\"min\": "));
    Serial.print(MIN_MAX_TEMP, DEC);
    Serial.print(F(", \"max\": "));
    Serial.print(MAX_MAX_TEMP, DEC);
    Serial.print(F("}, \"current\": "));
    Serial.print(maxfan_temp, DEC);
    Serial.print(F("}, \"polling\": {\"help\": \"temperature polling interval (ms)\""
        ", \"range\": {\"min\": "));
    Serial.print(MIN_POLL_INTERVAL, DEC);
    Serial.print(F(", \"max\": "));
    Serial.print(MAX_POLL_INTERVAL, DEC);
    Serial.print(F("}, \"current\": "));
    Serial.print(poll_ms, DEC);
    Serial.print(F("}, \"errlimit\": {\"help\": \"number of reading erros before failing to max speed\""
        ", \"range\": {\"min\": "));
    Serial.print(MIN_MAX_ERRORS, DEC);
    Serial.print(F(", \"max\": "));
    Serial.print(MAX_MAX_ERRORS, DEC);
    Serial.print(F("}, \"current\": "));
    Serial.print(max_errors, DEC);
    Serial.print(F("}, \"divider\": {\"help\": \"fan sensor divider\""
        ", \"values\": {\"2\": \"unipole hall effect sensor\""
        ", \"8\": \"bipole hall effect sensor\"}"
        ", \"range\": {\"min\": "));
    Serial.print(MIN_FAN_DIV, DEC);
    Serial.print(F(", \"max\": "));
    Serial.print(MAX_FAN_DIV, DEC);
    Serial.print(F("}, \"current\": "));
    Serial.print(fan_div, DEC);
    Serial.print(F("}, \"stats\": {\"help\": \"monitor mode\""
        ", \"values\": {\"0\": \"disabled\""
        ", \"1\": \"one time\""
        ", \"2\": \"continuous\"}, \"current\": "));
    Serial.print(stats, DEC);
    Serial.print(F("}, \"echo\": {\"help\": \"echo serial input back\""
        ", \"values\": {\"0\": \"disabled\""
        ", \"1\": \"enabled\"}, \"current\": "));
    Serial.print(serial_echo, DEC);
    Serial.print(F("}, \"uptime\": {\"help\": \"uptime (ms)\"}, \"current\": "));
    Serial.print(millis(), DEC);
    Serial.println(F("}"));
}

void set_value(char *param, long value) {
    if (strcmp(param, "mode") == 0) {
        mode = constrain(value, 0, 2);
        Serial.print(F("{\"mode\": "));
        Serial.print(mode);
        Serial.println(F("}"));
    } else if (strcmp(param, "start") == 0) {
        value = constrain(value, MIN_START_TEMP, MAX_START_TEMP);
        if (value >= maxfan_temp) {
            Serial.print(F("{\"start\": "));
            Serial.print(start_temp, DEC);
            Serial.println(F(", \"error\": \"Starting temperature can't be equal to or exceed threshold value\"}"));
        } else {
            start_temp = value;

            delta_temp = maxfan_temp - start_temp;
            log_b = log(255 / 0.4) / (float)delta_temp;
            log_a = (float)255 / exp(log_b * maxfan_temp);
            exp_a = (255 - 0.4) / log((float)maxfan_temp / start_temp);
            exp_b = exp(255 / exp_a) / (float)maxfan_temp;

            Serial.print(F("{\"start\": "));
            Serial.print(start_temp, DEC);
            Serial.println(F("}"));
        }
    } else if (strcmp(param, "max") == 0) {
        value = constrain(value, MIN_MAX_TEMP, MAX_MAX_TEMP);
        if (value <= start_temp) {
            Serial.print(F("{\"threshold\": "));
            Serial.print(maxfan_temp, DEC);
            Serial.println(F(", \"error\": \"Threshold temperature can't be equal to or lower than starting temperature\"}"));
        } else {
            maxfan_temp = value;

            delta_temp = maxfan_temp - start_temp;
            log_b = log(255 / 0.4) / (float)delta_temp;
            log_a = (float)255 / exp(log_b * maxfan_temp);
            exp_a = (255 - 0.4) / log((float)maxfan_temp / start_temp);
            exp_b = exp(255 / exp_a) / (float)maxfan_temp;

            Serial.print(F("{\"threshold\": "));
            Serial.print(maxfan_temp, DEC);
            Serial.println(F("}"));
        }
    } else if (strcmp(param, "poll") == 0) {
        poll_ms = (unsigned long)constrain(value, MIN_POLL_INTERVAL, MAX_POLL_INTERVAL);
        Serial.print(F("{\"polling\": "));
        Serial.print(poll_ms, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "div") == 0) {
        fan_div = constrain(value, MIN_FAN_DIV, MAX_FAN_DIV);
        Serial.print(F("{\"divider\": "));
        Serial.print(fan_div, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "err") == 0) {
        max_errors = constrain(value, MIN_MAX_ERRORS, MAX_MAX_ERRORS);
        Serial.print(F("{\"errlimit\": "));
        Serial.print(max_errors, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "stats") == 0) {
        stats = constrain(value, 0, 2);
        Serial.print(F("{\"stats\": "));
        Serial.print(stats, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "echo") == 0) {
        serial_echo = constrain(value, 0, 1);
        Serial.print(F("{\"echo\": "));
        Serial.print(serial_echo, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "info") == 0)
        show_info();
    else if (strcmp(param, "help") == 0)
        show_help();
}

void get_value(char *param) {
    if (strcmp(param, "mode") == 0) {
        Serial.print(F("{\"mode\": "));
        Serial.print(mode, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "start") == 0) {
        Serial.print(F("{\"start\": "));
        Serial.print(start_temp, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "max") == 0) {
        Serial.print(F("{\"threshold\": "));
        Serial.print(maxfan_temp, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "poll") == 0) {
        Serial.print(F("{\"polling\": "));
        Serial.print(poll_ms, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "err") == 0) {
        Serial.print(F("{\"errlimit\": "));
        Serial.print(max_errors, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "div") == 0) {
        Serial.print(F("{\"divider\": "));
        Serial.print(fan_div, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "stats") == 0) {
        if (stats == 0)
            stats = 1;
        else {
            Serial.print(F("{\"stats\": "));
            Serial.print(stats);
            Serial.println(F("}"));
        }
    } else if (strcmp(param, "echo") == 0) {
        Serial.print(F("{\"echo\": "));
        Serial.print(serial_echo, DEC);
        Serial.println(F("}"));
    } else if (strcmp(param, "info") == 0)
        show_info();
    else if (strcmp(param, "help") == 0)
        show_help();
}

/*
 * The setup function starts the sensors and inits pins
 */
void setup(void) {
    // configure I/O pins
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(RPM_PIN, INPUT);

    // start serial port
    Serial.begin(9600);
    Serial.println(F("{\"info\": \"Microserver Fan Controller v"
        VERSION
        "\"}"));
    show_info();

    // start up the library
    sensors.begin();

    // you can have more than one IC on the same bus; 0 refers to the first IC on the wire
    if (!sensors.getAddress(sensor, 0)) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println(F("{\"error\": \"Unable to find address for sensor\"}"));
        Serial.flush();
        analogWrite(PWM_PIN, 255);
        error_blink(SENSOR_NOTFOUND);
    }

    attachInterrupt(0, count_rotations, RISING);
}

/*
 * Main function, gets temperature, sets fan speed, processes commands
 */
void loop(void) {
    float temp;
    int pwm = 255;
    unsigned long start_time = millis();

    // check for input
    if (string_complete) {
        // read each command pair
        char* command = strtok(input_string, ",");
        while (command != 0)
        {
            // split the command in two values
            char* separator = strchr(command, '=');
            if (separator != 0) {
                *separator = '\0';
                // change runtime params
                set_value(command, atol(separator + 1));
            } else
                get_value(command);
            // find the next command in input string
            command = strtok(0, ",");
        }
        string_complete = false;
    }

    // send the command to get temperatures
    sensors.requestTemperatures();
    temp = sensors.getTempC(sensor);

    if (temp == DEVICE_DISCONNECTED_C) {
        Serial.println(F("{\"error\": \"Unable to read temperature from sensor\"}"));
        err_count++;
        if (err_count == max_errors) {
            analogWrite(PWM_PIN, pwm);
            error_blink(SENSOR_READERROR);
        }
    } else {
        if (err_count > 0)
            err_count--;

        if (temp < start_temp)
            pwm = 0;
        else if (temp >= maxfan_temp)
            pwm = 255;
        else
            switch(mode) {
                case 0: // logarithmic
                    pwm = log_a * exp(log_b * temp);
                    break;
                case 2: // exponential
                    pwm = exp_a * log(exp_b * temp);
                    break;
                default: // linear
                    pwm = (float)255 * (temp - start_temp) / delta_temp;
            }
    
        // set fan speed
        analogWrite(PWM_PIN, pwm);
    
        // turn led on when at full speed
        digitalWrite(LED_BUILTIN, (pwm == 255));
    }

    // print stats
    if (stats) {
        Serial.print("{\"temp\": ");
        Serial.print(temp);
        Serial.print(", \"rpm\": ");
        Serial.print((float)60 * 1000 / poll_ms * rotations / fan_div, 0);
        Serial.print(", \"pwm\": ");
        Serial.print(pwm);
        Serial.print(", \"err\": ");
        Serial.print(err_count);
        Serial.println("}");
        if (stats == 1)
            stats = 0;
    }

    // reset rpm count
    rotations = 0;

    // sleep a while
    delay(min(poll_ms - millis() + start_time, poll_ms));
}

/*
 * SerialEvent occurs whenever a new data comes in the
 * hardware serial RX; multiple bytes of data may be available
 */
void serialEvent() {
    while (Serial.available()) {
        // get the new byte
        char chr = (char)Serial.read();
        // if the incoming character is a newline or buffer is full, set a flag
        if ((chr == '\r') || (string_pos > MAX_INPUT_SIZE)) {
            input_string[string_pos] = '\0';
            string_pos = 0;
            string_complete = true;
        } else { // else add it to the inputString
            input_string[string_pos++] = chr;
        }
    }
    if (!string_complete)
        input_string[string_pos] = '\0';
    if (serial_echo)
        Serial.println(input_string);
}

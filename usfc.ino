/*
 * Copyright (c) 2017, Theodor-Iulian Ciobanu
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

#define VERSION "0.2.0"

// error codes
#define SENSOR_NOTFOUND     0xD
#define SENSOR_READERROR    0xB

// data wire is plugged into port 4 on the Arduino
#define ONE_WIRE_BUS 4
// PWM pin to fan
#define PWM_PIN 3
// RPM pin from fan
#define RPM_PIN 2

// polling interval
unsigned long poll_ms = 1000;

// temperatures to start fan and max speed
char start_temp = 32;
char maxfan_temp = 72;

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
// fan divider: 2 - unipole hall efect sensor, 8 - bipole hall efect sensor
unsigned char fan_div = 2;

// monitor mode: 0 - disabled, 1 - one time, 2 - continous
unsigned char stats = 2;

// echo serial input back
unsigned char serial_echo = 0;

// number of failed reads
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

    exit(-SENSOR_NOTFOUND);
}

void show_info() {
    Serial.print("{\"version\": ");
    Serial.print(VERSION);
    Serial.print(", \"start\": ");
    Serial.print(start_temp, DEC);
    Serial.print(", \"threshold\": ");
    Serial.print(maxfan_temp, DEC);
    Serial.print(", \"polling\": ");
    Serial.print(poll_ms);
    Serial.print(", \"errlimit\": ");
    Serial.print(max_errors);
    Serial.print(", \"divider\": ");
    Serial.print(fan_div);
    Serial.print(", \"stats\": ");
    Serial.print(stats);
    Serial.print(", \"echo\": ");
    Serial.print(serial_echo);
    Serial.print(", \"uptime\": ");
    Serial.print(millis());
    Serial.println("}");
}

void set_value(char *param, long value) {
    if (strcmp(param, "start") == 0) {
        value = constrain(value, 0, 64);
        if (value >= maxfan_temp) {
            Serial.print("{\"start\": ");
            Serial.print(start_temp, DEC);
            Serial.println(", \"error\": \"Starting temperature can't be equal to or exceed threshold value\"}");
        } else {
            start_temp = value;
            Serial.print("{\"start\": ");
            Serial.print(start_temp, DEC);
            Serial.println("}");
        }
    } else if (strcmp(param, "max") == 0) {
        value = constrain(value, 0, 128);
        if (value <= start_temp) {
            Serial.print("{\"threshold\": ");
            Serial.print(maxfan_temp, DEC);
            Serial.println(", \"error\": \"Threshold temperature can't be equal to or lower than starting temperature\"}");
        } else {
            maxfan_temp = value;
            Serial.print("{\"threshold\": ");
            Serial.print(maxfan_temp, DEC);
            Serial.println("}");
        }
    } else if (strcmp(param, "poll") == 0) {
        poll_ms = (unsigned long)constrain(value, 0, 3600);
        Serial.print("{\"polling\": ");
        Serial.print(poll_ms);
        Serial.println("}");
    } else if (strcmp(param, "div") == 0) {
        fan_div = constrain(value, 1, 16384);
        Serial.print("{\"divider\": ");
        Serial.print(fan_div);
        Serial.println("}");
    } else if (strcmp(param, "err") == 0) {
        max_errors = constrain(value, 1, 3600);
        Serial.print("{\"errlimit\": ");
        Serial.print(max_errors);
        Serial.println("}");
    } else if (strcmp(param, "stats") == 0) {
        stats = constrain(value, 0, 2);
        Serial.print("{\"stats\": ");
        Serial.print(stats);
        Serial.println("}");
    } else if (strcmp(param, "echo") == 0) {
        serial_echo = constrain(value, 0, 1);
        Serial.print("{\"echo\": ");
        Serial.print(serial_echo);
        Serial.println("}");
    } else if (strcmp(param, "info") == 0)
        show_info();
}

void get_value(char *param) {
    if (strcmp(param, "start") == 0) {
        Serial.print("{\"start\": ");
        Serial.print(start_temp, DEC);
        Serial.println("}");
    } else if (strcmp(param, "max") == 0) {
        Serial.print("{\"threshold\": ");
        Serial.print(maxfan_temp, DEC);
        Serial.println("}");
    } else if (strcmp(param, "poll") == 0) {
        Serial.print("{\"polling\": ");
        Serial.print(poll_ms);
        Serial.println("}");
    } else if (strcmp(param, "err") == 0) {
        Serial.print("{\"errlimit\": ");
        Serial.print(max_errors);
        Serial.println("}");
    } else if (strcmp(param, "div") == 0) {
        Serial.print("{\"divider\": ");
        Serial.print(fan_div);
        Serial.println("}");
    } else if (strcmp(param, "stats") == 0) {
        if (stats == 0)
            stats = 1;
        else {
            Serial.print("{\"stats\": ");
            Serial.print(stats);
            Serial.println("}");
        }
    } else if (strcmp(param, "echo") == 0) {
        Serial.print("{\"echo\": ");
        Serial.print(serial_echo);
        Serial.println("}");
    } else if (strcmp(param, "info") == 0)
        show_info();
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
    Serial.print("{\"info\": \"Microserver Fan Controller v");
    Serial.print(VERSION);
    Serial.println("\"}");
    show_info();

    // start up the library
    sensors.begin();

    // you can have more than one IC on the same bus; 0 refers to the first IC on the wire
    if (!sensors.getAddress(sensor, 0)) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("{\"error\": \"Unable to find address for sensor\"}");
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
    int pwm = 0;
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
        Serial.println("{\"error\": \"Unable to read temperature from sensor\"}");
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
            pwm = 255.0 * (temp - start_temp) / (maxfan_temp - start_temp);
    
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
        Serial.print(60.0 * 1000 / poll_ms * rotations / fan_div);
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

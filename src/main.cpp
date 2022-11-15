// vim: sta:et:sw=4:ts=4:sts=4
#include <Arduino.h>

// https://github.com/ppedro74/Arduino-SerialCommands
#include <SerialCommands.h>

const int button_pin = 2; // PD2
const int LASER_OUT = 3; // PD3

const int laser_period = 80; // in us

/*
   Red connector 

   ----  ------------------------------
   | +   SIG (tie to GND to OFF laser  |
   | -   GND                           |
   ------------------------------------

*/

bool pulse_laser = true;

//unsigned long nextPtime = 0;
char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

//This is the default handler, and gets called when no other command matches.
void cmd_unrecognized(SerialCommands* sender, const char* cmd) {
    sender->GetSerial()->print("Unrecognized command [");
    sender->GetSerial()->print(cmd);
    sender->GetSerial()->println("]");
}

//called for ON command
void cmd_led_on(SerialCommands* sender) {
    pulse_laser = true;
    digitalWrite(LED_BUILTIN, pulse_laser);
    sender->GetSerial()->println("Pulse is on");
}

//called for OFF command
void cmd_led_off(SerialCommands* sender) {
    pulse_laser = false;
    digitalWrite(LED_BUILTIN, LOW);
    sender->GetSerial()->println("Pulse is off");
}

//Note: Commands are case sensitive
SerialCommand cmd_led_on_("ON", cmd_led_on);
SerialCommand cmd_led_off_("OFF", cmd_led_off);
//SerialCommand cmd_datetime_set_("TS", cmd_datetime_set); // requires one argument
/// Add one_key commands
SerialCommand cmd_ledo_("o", cmd_led_on, true);
SerialCommand cmd_ledf_("f", cmd_led_off, true);

unsigned long nextLtime, nextBtime;
bool laser_state=true;

//delay(1);
void setup() {

    // initialize LED digital pin as an input (Laser ON).
    pinMode(LASER_OUT, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    pinMode(button_pin, INPUT_PULLUP);
    // Start serial port
    Serial.begin(115200);
    serial_commands_.SetDefaultHandler(cmd_unrecognized);
    serial_commands_.AddCommand(&cmd_led_on_);
    serial_commands_.AddCommand(&cmd_led_off_);
    serial_commands_.AddCommand(&cmd_ledf_);
    serial_commands_.AddCommand(&cmd_ledo_);
//    serial_commands_.AddCommand(&cmd_print_);

    Serial.println(F("Ola Mundo!"));
    nextLtime = 0;
    nextBtime = 0;
}

void laser_off() {
    pinMode(LASER_OUT, OUTPUT);
    digitalWrite(LASER_OUT, LOW);
}

void laser_pulse(unsigned long now_us) {
    if (pulse_laser){

        if ( now_us > nextLtime ) {
            if(laser_state){
                pinMode(LASER_OUT, INPUT_PULLUP); // laser ON
            }
            else {
                laser_off();
            }
            nextLtime += laser_period / 2;
            laser_state = not laser_state;  // Toggle
        }
    }
    else {
        //pinMode(LASER_OUT, INPUT_PULLUP);
        laser_state=false;
        laser_off();
    }
}

void read_button(unsigned long now_ms) {
        if ( now_ms > nextBtime ) {
            if (!digitalRead(button_pin)){
                pulse_laser = not pulse_laser;
                digitalWrite(LED_BUILTIN, pulse_laser);
                nextBtime += 500;  // in millis
            }
        }

}
void loop() {
    unsigned long us = micros();
    laser_pulse(us);
    unsigned long ms = millis();
    read_button(ms);
    serial_commands_.ReadSerial();
}

// vim: sta:et:sw=4:ts=4:sts=4
#include <Arduino.h>

// https://github.com/ppedro74/Arduino-SerialCommands
#include <SerialCommands.h>

const int button_pin = 2; // PD2
const int LASER_OUT = 3; // PD3
// -- Only A0-A5 can be Digital I/O
const int RED_LED = A4; // A7;
const int GREEN_LED = A3;
const int BLUE_LED = A5; // A7;

const int laser_period = 80; // in us

/*
   Red connector 

   ----  ------------------------------
   | +   SIG (tie to GND to OFF laser  |
   | -   GND                           |
   ------------------------------------

*/

bool pulse_laser = true;
bool redOn = true, greenOn = true, blueOn = true;

//unsigned long nextPtime = 0;
char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

//This is the default handler, and gets called when no other command matches.
void cmd_unrecognized(SerialCommands* sender, const char* cmd) {
    sender->GetSerial()->print("Unrecognized command [");
    sender->GetSerial()->print(cmd);
    sender->GetSerial()->println("]");
}

void cmd_green_on(SerialCommands* sender) {
    greenOn = true;
    sender->GetSerial()->println("green led is on");
}
void cmd_green_off(SerialCommands* sender) {
    greenOn = false;
    sender->GetSerial()->println("green led is off");
}void cmd_blue_on(SerialCommands* sender) {
    blueOn = true;
    sender->GetSerial()->println("Blue led is on");
}
void cmd_blue_off(SerialCommands* sender) {
    blueOn = false;
    sender->GetSerial()->println("Blue led is off");
}
void cmd_red_on(SerialCommands* sender) {
    redOn = true;
    sender->GetSerial()->println("Red led is on");
}
void cmd_red_off(SerialCommands* sender) {
    redOn = false;
    sender->GetSerial()->println("Red led is off");
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
SerialCommand cmd_redo_("R", cmd_red_on, true);
SerialCommand cmd_redf_("r", cmd_red_off, true);
SerialCommand cmd_greeno_("G", cmd_green_on, true);
SerialCommand cmd_greenf_("g", cmd_green_off, true);
SerialCommand cmd_blueo_("B", cmd_blue_on, true);
SerialCommand cmd_bluef_("b", cmd_blue_off, true);
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

    pinMode(RED_LED, OUTPUT);
    digitalWrite(RED_LED, HIGH);
    pinMode(GREEN_LED, OUTPUT);
    digitalWrite(GREEN_LED, HIGH);
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(BLUE_LED, HIGH);

    pinMode(button_pin, INPUT_PULLUP);
    // Start serial port
    Serial.begin(115200);
    serial_commands_.SetDefaultHandler(cmd_unrecognized);
    serial_commands_.AddCommand(&cmd_led_on_);
    serial_commands_.AddCommand(&cmd_led_off_);
    serial_commands_.AddCommand(&cmd_ledf_);
    serial_commands_.AddCommand(&cmd_ledo_);
    serial_commands_.AddCommand(&cmd_redf_);
    serial_commands_.AddCommand(&cmd_redo_);
    serial_commands_.AddCommand(&cmd_greenf_);
    serial_commands_.AddCommand(&cmd_greeno_);
    serial_commands_.AddCommand(&cmd_bluef_);
    serial_commands_.AddCommand(&cmd_blueo_);
//    serial_commands_.AddCommand(&cmd_print_);

    Serial.println(F("Ola Mundo! o:pulse ON, f:pulse OFF, R:red ON, r:Red OFF"));
    nextLtime = 0;
    nextBtime = 0;
}

void laser_off() {
    pinMode(LASER_OUT, OUTPUT);
    digitalWrite(LASER_OUT, LOW);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BLUE_LED, HIGH);
}

void laser_pulse(unsigned long now_us) {
    if (pulse_laser){

        if ( now_us > nextLtime ) {
            if(laser_state){
                pinMode(LASER_OUT, INPUT_PULLUP); // laser ON
                if(redOn)
                    digitalWrite(RED_LED, LOW);
                if(greenOn)
                    digitalWrite(GREEN_LED, LOW);
                if(blueOn)
                    digitalWrite(BLUE_LED, LOW);
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

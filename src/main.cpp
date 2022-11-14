// vim: sta:et:sw=4:ts=4:sts=4
#include <Arduino.h>

// https://github.com/ppedro74/Arduino-SerialCommands
#include <SerialCommands.h>

#define LASER_OUT 2 // PD2  
const int button_pin = 3; // PD3
const int laser_period = 80; // in us

/*
   Red connector 

   ----  ----
   | 2   GND  |
   ------------

*/

bool pulse_led = false;

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
    pulse_led = true;
    digitalWrite(LED_BUILTIN, HIGH);
    sender->GetSerial()->println("Pulse is on");
}

//called for OFF command
void cmd_led_off(SerialCommands* sender) {
    pulse_led = false;
    digitalWrite(LED_BUILTIN, LOW);
    sender->GetSerial()->println("Pulse is off");
}
//Note: Commands are case sensitive
SerialCommand cmd_led_on_("ON", cmd_led_on);
SerialCommand cmd_led_off_("OFF", cmd_led_off);
//SerialCommand cmd_datetime_set_("TS", cmd_datetime_set); // requires one argument
/// Add one_key commands 
//SerialCommand cmd_print_("p", cmd_print, true);


//delay(1);
void setup() {

    // initialize LED digital pin as an input (Laser ON).
    pinMode(LASER_OUT, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    // Start serial port
    Serial.begin(115200);
    Serial.println("Ola Mundo");
}

void laser_pulse(unsigned long now_us) {
    static unsigned long nextLtime = 0;
    static bool led_state=true;
    if (pulse_led){

        if ( now_us > nextLtime ) {
            if(led_state){
                pinMode(LASER_OUT, INPUT_PULLUP); // laser ON
                nextLtime += laser_period / 2;

            }
            else {
                pinMode(LASER_OUT, OUTPUT);
                digitalWrite(LASER_OUT, LOW);
                nextLtime += laser_period / 2;
            }
        }
    }
    else
        pinMode(LASER_OUT, INPUT_PULLUP);
}

void laser_off() {
    pinMode(LASER_OUT, OUTPUT);
    digitalWrite(LASER_OUT, LOW);
}

void loop() {
    unsigned long us = micros();
    laser_pulse(us);
}

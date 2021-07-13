/* Author: 
 *  Fredi R. Mino
 * Date: 
 *  07-13-2021
 * 
 * Description:
 *  This code samples 5 analog channels at 222.2 Hz.
 *  It stores the information in a buffer, and prints it.
*/


#define BAUD_RATE 115200

#define CHANNEL_1 A1
#define CHANNEL_2 A2
#define CHANNEL_3 A3
#define CHANNEL_4 A4
#define CHANNEL_5 A5

#define SAMPLING_FREQ 222.2


#include <arduino-timer.h>

auto sampling_timer = timer_create_default();
int debug_counter = 0;

bool debug_timer(void *){
  Serial.print("I'm alive x");
  Serial.println(debug_counter++);
  return true;
}

void setup() {
  Serial.begin( BAUD_RATE);
  int period_ms = round( 1000.0 / SAMPLING_FREQ);
  sampling_timer.every( period_ms, debug_timer);
}

void loop() {
  sampling_timer.tick();
}

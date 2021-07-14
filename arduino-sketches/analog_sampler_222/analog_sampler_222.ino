/* Author: 
 *  Fredi R. Mino
 * Date: 
 *  07-13-2021
 * 
 * Description:
 *  This code samples 5 analog channels at 222.2 Hz.
 *  It stores the information in a buffer, and prints it.
*/

#include <arduino-timer.h>

#define BAUD_RATE 115200

#define SAMPLING_FREQ 222.2

#define CH_01 14
#define CH_02 15
#define CH_03 16
#define CH_04 17
#define CH_05 18


auto sampling_timer = timer_create_default();
int debug_counter = 0;
uint32_t temp_data;

bool debug_timer(void *){
  Serial.println("Data: ");
  for (int chan=CH_01; chan <= CH_05; chan++){
    temp_data = analogRead(chan);
    Serial.print(temp_data);
    Serial.print(", ");
  }
  Serial.println();  
  
  return true;
}

void setup() {
  Serial.begin( BAUD_RATE);
  int period_ms = round( 1000.0 / SAMPLING_FREQ);
  sampling_timer.every( period_ms, debug_timer);
  Serial.print("I'm alive");
}

void loop() {
  sampling_timer.tick();
}

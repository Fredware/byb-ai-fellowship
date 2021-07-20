/* Author: 
 *  Fredi R. Mino
 * Date: 
 *  07-16-2021
 * 
 * Description:
 *  This code samples 5 analog channels at 222.2 Hz.
 *  It stores the information in a buffer, and prints it.
*/
/*******/
/* Includes ---------------------------------------------------------------- */
#include "CircularBuffer.h"
#include <a5c5f-env-norm-55-007_inferencing.h>
#include <arduino-timer.h>
#include <Arduino_LSM9DS1.h>
#include <Servo.h>  // the servo library



/* 200Hz Analog Sampler ---------------------------------------------------- */
#define BAUD_RATE 115200
#define SAMPLING_FREQ 222.2

#define CH_01 14
#define CH_02 15
#define CH_03 16
#define CH_04 17
#define CH_05 18
/* ------------------------------------------------------------------------- */

#define N_CHANS 5
#define FRAME_LEN 20

/* Motor Control ----------------------------------------------------------- */
#define THUMB_IDX 4
#define INDEX_IDX 0
#define MIDDLE_IDX 1
#define RING_IDX 3
#define PINKY_IDX 2

#define OPEN 2
#define CLOSE 1

#define SERVOS 5 // the number of servos
#define SRV_CLOSE 0 // angle for closed finger
#define SRV_OPEN 180 // angle for open finger

#define BUFFER_SIZE 66

int finalReading1;
int finalReading2;
int finalReading3;
int finalReading4;

int servoPins[SERVOS] = {9,10,11,12,13}; // servos on pins 9 through 13 (thumb, index, middle, ring, pinky)
int servoMin[SERVOS] = {544,544,544,544,544}; // minimum pwm value for each servo
int servoMax[SERVOS] = {2400,2400,2400,2400,2400}; // maximum pwm value for each servo
int servoInv[SERVOS] = {0,0,0,0,0}; // indicates if servo direction is inverted

unsigned long tmsServo_Attached[SERVOS]; // time since last servo attach. Used to reduce power comsumption
unsigned long previousTime = 0;

Servo myservo[SERVOS];
/* ------------------------------------------------------------------------- */

/* Circular Buffer ---------------------------------------------------- */
unsigned long analog_read_time;

CircularBuffer<float, BUFFER_SIZE> buffer_ch01;
CircularBuffer<float, BUFFER_SIZE> buffer_ch02;
CircularBuffer<float, BUFFER_SIZE> buffer_ch03;
CircularBuffer<float, BUFFER_SIZE> buffer_ch04;
CircularBuffer<float, BUFFER_SIZE> buffer_ch05;
/* ------------------------------------------------------------------------- */

/* Dataset Statistics ------------------------------------------------------ */
float means[5] = { -14614.78766535,
                   -11954.29539468, 
                   -14115.51905832, 
                   -14441.05551792, 
                   -14464.99905807};
float stdevs[5] = { 1495.1199467,
                    1343.30603843,
                    2043.12148758,
                    1353.73643992,
                    386.24717924};
float norms[5] = { 16.91287963,
                   15.46845307,
                   12.58684762,
                   16.86506427,
                   8.17662202};

float alpha[5];
/* ------------------------------------------------------------------------- */

/* 200Hz Analog Sampler ---------------------------------------------------- */
auto sampling_timer = timer_create_default();
int debug_counter = 0;
uint32_t temp_data;
/* ------------------------------------------------------------------------- */

/* Inference Model --------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

static const float features[] = {
    // copy raw features here (for example from the 'Live classification' page)
    // see https://docs.edgeimpulse.com/docs/running-your-impulse-arduino
    -0.3809, -0.4746, -0.5160, -0.5489, -0.8806, -0.3134, -0.4810, -0.0593, -0.5490, -0.7550, -0.2870,
    -0.3899, 0.4217, -0.5493, -0.8335, -0.2843, -0.4189, 1.9378, -0.4997, -1.0188, -0.3034, -0.6140,
    3.7326, -0.2669, -1.0113, -0.1515, -0.5053, 4.4153, 0.0351, -0.4798, 0.1105, -0.4794, 5.5325,
    2.6580, 0.1328, 0.2387, -0.3932, 5.7938, 4.0086, 0.3559, 0.4816, -0.0272, 6.1214, 4.4982, 
    1.1336, 1.0228, 0.1696, 5.7815, 3.3322, 1.6830, 1.4567, 0.4759, 5.5055, 2.2049, 2.8162
};
/* ------------------------------------------------------------------------- */

/* 200Hz Analog Sampler ---------------------------------------------------- */
bool debug_timer(void *){
  Serial.println("ANALOG SAMPLING: START");
  Serial.print("\tData: ");
  for (int chan=CH_01; chan <= CH_05; chan++){
    temp_data = analogRead(chan);
    Serial.print(temp_data);
    if( chan != CH_05) Serial.print(", ");
  }
  Serial.println();
  Serial.println("ANALOG SAMPLING: DONE");  
  return true;
}
/* ------------------------------------------------------------------------- */

/* Inference Model --------------------------------------------------------- */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

/**
* @brief      Printf function uses vsnprintf and output using Arduino Serial
*
* @param[in]  format     Variable argument list
*/
void ei_printf(const char *format, ...) {
   static char print_buf[1024] = { 0 };

   va_list args;
   va_start(args, format);
   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
   va_end(args);

   if (r > 0) {
       Serial.write(print_buf);
   }
}
/* ------------------------------------------------------------------------- */


/* Motor Control ----------------------------------------------------------- */
void moveFinger(int finger, int pos)
{
  if (pos == 1) { // Close
    if (myservo[finger].read() != abs(SRV_CLOSE - SRV_OPEN*servoInv[finger])) {
      // Close finger
      if (!myservo[finger].attached()) {
        myservo[finger].attach(servoPins[finger], servoMin[finger], servoMax[finger]);
        tmsServo_Attached[finger] = millis();
      }
      myservo[finger].write(abs(SRV_CLOSE - SRV_OPEN*servoInv[finger]));
    }
  }
  else { // Open
    if (myservo[finger].read() != abs(SRV_OPEN - SRV_OPEN*servoInv[finger])) {
      // Open finger
      if (!myservo[finger].attached()) {
        myservo[finger].attach(servoPins[finger], servoMin[finger], servoMax[finger]);
        tmsServo_Attached[finger] = millis();
      }
      myservo[finger].write(abs(SRV_OPEN - SRV_OPEN*servoInv[finger]));
    }
  }
}
/* ------------------------------------------------------------------------- */

/*****************************************************************************/
/*****************************************************************************/
/********************************************************************* SETUP */
/*****************************************************************************/
/*****************************************************************************/
void setup() {
  /* ----------------------------------------------------------------------- */
  Serial.begin( BAUD_RATE);
  /* ----------------------------------------------------------------------- */
  for (int f = 0; f < SERVOS; f++) moveFinger (f, OPEN);  // Open Hand
  delay (1000);                                            // Wait for servos
  for (int f = 0; f < SERVOS; f++) myservo[f].detach();   // Relax servos
  delay (2000);
  /* ----------------------------------------------------------------------- */
  for( int i = 0; i < N_CHANS; i++){
    alpha[i] = FRAME_LEN * stdevs[i] * norms[i];
    alpha[i] = 1.0 / alpha[i];
//    Serial.print(alpha[i], 7);
//    Serial.print(" ");
  }
//  Serial.println();
  /* ----------------------------------------------------------------------- */
//  Serial.println("====================================");
//  Serial.println("=== Hacker Hand Integration Demo ===");
//  Serial.println("====================================");
  /* ----------------------------------------------------------------------- */
  analog_read_time = millis();  //Initialize reference timestamp
  /* ----------------------------------------------------------------------- */
}

/*****************************************************************************/
/*****************************************************************************/
/********************************************************************** LOOP */
/*****************************************************************************/
/*****************************************************************************/
void loop() {
  
  float temp_data = 0;
  float sum[5]={0,0,0,0,0};
  
  for (int i = 0; i < FRAME_LEN; i++){
    for(int j = 0; j < N_CHANS; j++){
      temp_data = analogRead(CH_01+j);
//      if (temp_data>10000) temp_data = 10000;
      temp_data = temp_data - means[j];
      temp_data = abs(temp_data);
      sum[j]+= temp_data;
      if( i == ( FRAME_LEN -1)){
        sum[j] *= alpha[j];
        Serial.print(sum[j]);
        Serial.print(",");
      }
    }
    Serial.println();
    delay(4);
  }

//  for(int j = 0; j < N_CHANS; j++){
//  }
//  Serial.print(-0.5);
//  Serial.print(" ");
//  for (int chan=CH_01; chan <= CH_05; chan++){
//    temp_data = analogRead(chan);
//    switch(chan){
//      case CH_01:
//        buffer_ch01.push(temp_data);
//        Serial.print(buffer_ch01.last());
//        Serial.print(", ");
//        break;
//      case CH_02:
//        buffer_ch02.push(temp_data);
//        Serial.print(buffer_ch02.last());
//        Serial.print(", ");
//        break;
//      case CH_03:
//        buffer_ch03.push(temp_data);
//        Serial.print(buffer_ch03.last());
//        Serial.print(", ");
//        break;
//      case CH_04:
//        buffer_ch04.push(temp_data);
//        Serial.print(buffer_ch04.last());
//        Serial.print(", ");
//        break;
//      case CH_05:
//        buffer_ch05.push(temp_data);
//        Serial.print(buffer_ch05.last());
//        Serial.println();
//        break;
//      default:
//        Serial.println("\t\tERROR DURING ANALOG SAMPLING");
//        break;
//    }
//  }
//  }
//  Serial.println("\tANALOG SAMPLING: DONE");
 
//  /* ----------------------------------------------------------------------- */
//  if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
//    ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
//              EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
//    delay(1000);
//    return;
//  }
// 
//  ei_impulse_result_t result = { 0 };
//  
//  // the features are stored into flash, and we don't want to load everything into RAM
//  signal_t features_signal;
//  features_signal.total_length = sizeof(features) / sizeof(features[0]);
//  features_signal.get_data = &raw_feature_get_data;
//
//  Serial.println("\tCLASSIFICATION: START");
//  // invoke the impulse
//  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
//
//  if (res != 0) return;
//
//  // human-readable predictions
//  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
//    ei_printf("\t\t    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
//  }
//  #if EI_CLASSIFIER_HAS_ANOMALY == 1
//  ei_printf("    anomaly score: %.3f\n", result.anomaly);
//  #endif
//  Serial.println("\tCLASSIFICATION: END");  
//  
//  /* ------------------------------------------------------------------------- */
//  /*Get the prediction with the highest probability*/
//  Serial.println("\tCLASS SELECTION: START");
//  float max_val = 0;
//  size_t max_idx = 0;
//  for( size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++){
//    if (i==0){
//      max_val = result.classification[i].value;
//      max_idx = 0;
//    }
//    if(result.classification[i].value > max_val){
//      max_val = result.classification[i].value;
//      max_idx = i;
//    }
//  }
//  Serial.print("\t\tPREDICTED CLASS: ");
//  Serial.println(result.classification[max_idx].label);
//  Serial.print("\t\tCONFIDENCE: ");
//  Serial.println(max_val);
//  Serial.print("\t\tINDEX: ");
//  Serial.println(max_idx);
//  Serial.println("\tCLASS SELECTION: END");
//  
//  /* ------------------------------------------------------------------------- */
//  // Detach servo if time since last attach exceeds 500 ms.
//  for (int i=0; i < SERVOS; i++) {
//    if (millis() - tmsServo_Attached[i] > 500) {
//      myservo[i].detach();
//    }
//  }
//
//  Serial.println("\tMOTOR CONTROL: START");
//  switch(max_idx){
//    case THUMB_IDX:
//      moveFinger (0, CLOSE);
//      Serial.println("\t\tHAND STATUS: CLOSED");
//      delay(1000);
//      moveFinger (0, OPEN);
//      Serial.println("\t\tHAND STATUS: OPEN");
//      delay(1000);
//      break;
//    case INDEX_IDX:
//      moveFinger (1, CLOSE);
//      Serial.println("\t\tHAND STATUS: CLOSED");
//      delay(1000);
//      moveFinger (1, OPEN);
//      Serial.println("\t\tHAND STATUS: OPEN");
//      delay(1000);      
//      break;
//    case MIDDLE_IDX:
//      moveFinger (2, CLOSE);
//      Serial.println("\t\tHAND STATUS: CLOSED");
//      delay(1000);
//      moveFinger (2, OPEN);
//      Serial.println("\t\tHAND STATUS: OPEN");
//      delay(1000);    
//      break;
//    case RING_IDX:
//      moveFinger (3, CLOSE);
//      Serial.println("\t\tHAND STATUS: CLOSED");
//      delay(1000);
//      moveFinger (3, OPEN);
//      Serial.println("\t\tHAND STATUS: OPEN");
//      delay(1000);
//      break;
//    case PINKY_IDX:
//      moveFinger (4, CLOSE);
//      Serial.println("\t\tHAND STATUS: CLOSED");
//      delay(1000);
//      moveFinger (4, OPEN);
//      Serial.println("\t\tHAND STATUS: OPEN");
//      delay(1000);
//      break;
//    default:
//      Serial.println("ERROR DURING MOTOR CONTROL (x_x)");
//      break;
//  }
//  Serial.println("\tMOTOR CONTROL: END");
//  
//  Serial.println("<--- THE LOOP ENDS HERE --->");
}

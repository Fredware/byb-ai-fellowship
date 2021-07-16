/* Author: 
 *  Fredi R. Mino
 * Date: 
 *  07-13-2021
 * 
 * Description:
 *  This code samples 5 analog channels at 222.2 Hz.
 *  It stores the information in a buffer, and prints it.
*/
/*******/
/* Includes ---------------------------------------------------------------- */
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

/* Motor Control ----------------------------------------------------------- */
#define SERVOS 5 // the number of servos
#define SRV_CLOSE 0 // angle for closed finger
#define SRV_OPEN 180 // angle for open finger

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
    -0.3809, -0.4746, -0.5160, -0.5489, -0.8806, -0.3134, -0.4810, -0.0593, -0.5490, -0.7550, -0.2870, -0.3899, 0.4217, -0.5493, -0.8335, -0.2843, -0.4189, 1.9378, -0.4997, -1.0188, -0.3034, -0.6140, 3.7326, -0.2669, -1.0113, -0.1515, -0.5053, 4.4153, 0.0351, -0.4798, 0.1105, -0.4794, 5.5325, 2.6580, 0.1328, 0.2387, -0.3932, 5.7938, 4.0086, 0.3559, 0.4816, -0.0272, 6.1214, 4.4982, 1.1336, 1.0228, 0.1696, 5.7815, 3.3322, 1.6830, 1.4567, 0.4759, 5.5055, 2.2049, 2.8162
};
/* ------------------------------------------------------------------------- */

/* 200Hz Analog Sampler ---------------------------------------------------- */
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
/********************************************************************* SETUP */
/*****************************************************************************/
void setup() {
  Serial.begin( BAUD_RATE);
  int period_ms = round( 1000.0 / SAMPLING_FREQ);
  sampling_timer.every( period_ms, debug_timer);
  Serial.print("I'm alive");
  /*******/
  // put your setup code here, to run once:
//    Serial.begin(115200);
    Serial.println("Edge Impulse Inferencing Demo");
    /****/
/***POCBLE**/
  // Open Hand
  for (int f=0; f < SERVOS; f++)
    moveFinger (f, 0);

  // Wait for servos to respond
  delay (500);

  // Relax servos
  for (int f=0; f < SERVOS; f++)
    myservo[f].detach();

  delay (2000);
/***/
}

/*****************************************************************************/
/********************************************************************** LOOP */
/*****************************************************************************/
void loop() {
  sampling_timer.tick();
  /* ----------------------------------------------------------------------- */
  ei_printf("Edge Impulse standalone inferencing (Arduino)\n");
  
  if (sizeof(features) / sizeof(float) != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
    ei_printf("The size of your 'features' array is not correct. Expected %lu items, but had %lu\n",
              EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, sizeof(features) / sizeof(float));
    delay(1000);
    return;
  }
  
  ei_impulse_result_t result = { 0 };
  
  // the features are stored into flash, and we don't want to load everything into RAM
  signal_t features_signal;
  features_signal.total_length = sizeof(features) / sizeof(features[0]);
  features_signal.get_data = &raw_feature_get_data;
  
  // invoke the impulse
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
  ei_printf("run_classifier returned: %d\n", res);

  if (res != 0) return;
  
  // print the predictions
  ei_printf("Predictions ");
  ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
             result.timing.dsp, result.timing.classification, result.timing.anomaly);
  ei_printf(": \n");
  ei_printf("[");
  
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("%.5f", result.classification[ix].value);
    #if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf(", ");
    #else
    if (ix != EI_CLASSIFIER_LABEL_COUNT - 1) {
      ei_printf(", ");
    }
    #endif
  }
  
  #if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("%.3f", result.anomaly);
  #endif
  ei_printf("]\n");

  // human-readable predictions
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
  }
  #if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
  #endif
  
  delay(1000);
  /* ------------------------------------------------------------------------- */
  unsigned long time=0;
  
  finalReading1 = analogRead(A0);
  finalReading2 = analogRead(A1);
  finalReading3 = analogRead(A2);
  finalReading4 = analogRead(A4);
  
  delay(10);

  for (int i=0; i < SERVOS; i++) {
    // Detach servo if time since last attach exceeds 750 ms.
    if (millis() - tmsServo_Attached[i] > 750) {
      myservo[i].detach();
    }
  }

  time = millis(); 
  if (finalReading1>500){
    moveFinger (1, 1);
    moveFinger (4, 1);
    moveFinger (5, 1);
    delay(3000);
    moveFinger (1, 0);
    moveFinger (4, 0);
    moveFinger (5, 0);
//    myservo[1].detach();
//    myservo[4].detach();
//    myservo[5].detach();
    delay(3000);
  }
}

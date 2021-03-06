/* Author:
    Fredi R. Mino
   Date:
    08-22-2021

   Description:
    This code samples 5 analog channels at 222.2 Hz.
    It stores the information in a buffer, and prints it.
*/

/* Libraries ---------------------------------------------------------------- */
#include "CircularBuffer.h"
#include <final_v3_nano_55_inferencing.h>
#include <arduino-timer.h>
#include <Arduino_LSM9DS1.h>
#include <Servo.h>

/* Analog Sampling ---------------------------------------------------------- */
#define BAUD_RATE 115200
#define SAMPLING_FREQ 222.2
#define SAMPLING_DELAY 3750

#define N_CHANS 5

#define CH_01 14
#define CH_02 15
#define CH_03 16
#define CH_04 17
#define CH_05 18

/* DSP --------------------------------------------------------------------- */
#define FRAME_LEN 20
#define STEP_LEN 6
#define FEATURE_LEN 11

#define THRESHOLD 0.2065
#define D_THRESHOLD 0.1375

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


int servoPins[SERVOS] = {9, 10, 11, 12, 13}; // servos pins (thumb, index, middle, ring, pinky)
int servoMin[SERVOS] = {1000, 544, 544, 544, 544}; // min pwm val for each servo
int servoMax[SERVOS] = {1750, 2400, 2400, 2400, 2400}; // max pwm val for each servo
int servoInv[SERVOS] = {0, 0, 0, 0, 0}; // indicates if servo direction is inverted

unsigned long tmsServo_Attached[SERVOS]; // time since last servo attach. Used to reduce power comsumption

Servo myservo[SERVOS];

/* Dataset Statistics ------------------------------------------------------ */
const float means[5] = {1.712837838,  90.64527027, 0.9189189189,  7.945945946, 1.722972973};

const float norms[5] = { 516.29,  932.35,  547.08,  966.05,  1021.28};

float alpha_coeffs[5];

/* Inference Model --------------------------------------------------------- */
float features [55] = {0};

/* Filtering ---------------------------------------------------------------
   filter formula: https://www.earlevel.com/main/2003/02/28/biquads/
                   y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] ??? b1*y[n-1] ??? b2*y[n-2]
   coefficient calculator: https://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/
*/

// Low-pass filter
// 222.2 Hz Sample Rate
// 8 Hz Cutoff frequency
// 0.7071 Q
// 6 dB gain.
const float a0 = 0.010995248654511658;
const float a1 = 0.021990497309023315;
const float a2 = 0.010995248654511658;
const float b1 = -1.6822395838277024;
const float b2 = 0.7262205784457494;

float x[N_CHANS][3];
float y[N_CHANS][2];
float filteredData01[5];

// call this on each new sample
void filterData01( float newValue[]) {
  for (int j = 0; j < N_CHANS; j++) {
    x[j][2] = x[j][1];
    x[j][1] = x[j][0];
    x[j][0] = newValue[j];
    filteredData01[j] = a0 * x[j][0] + a1 * x[j][1] + a2 * x[j][2] - b1 * y[j][0] - b2 * y[j][1];
    y[j][1] = y[j][0];
    y[j][0] = filteredData01[j];
  }
}

/*****************************************************************************
************************************ SETUP **********************************
*****************************************************************************/
size_t max_idx = 0;

// State variables to control when classification and control happen
char classification_flag = 0;
char hacking_flag = 0;
char debounce_flag = 0;

// Make the buffer slightly bigger than your frame length.
// 2 is the smallest number that is guaranteed to return an integer for the buffer size.
CircularBuffer<float, 2 * FRAME_LEN> env_data_buff [5];
CircularBuffer<float, FEATURE_LEN> mav_feats_buff [5];

void setup() {
  Serial.begin( BAUD_RATE);
  delay(10000);
  initialize_hand();
  compute_alpha_coeffs();

  for ( int i = 0; i < FRAME_LEN; i++) {
    for ( int chan = 0; chan < N_CHANS; chan++) {
      env_data_buff[chan].push( analogRead( CH_01 + chan));
    }
    delayMicroseconds(SAMPLING_DELAY);
  }
}

/******************************************************************************/
/************************************ LOOP ************************************/
/******************************************************************************/
void loop() {
  for ( int i = 0; i < STEP_LEN; i++) {
    for ( int chan = 0; chan < N_CHANS; chan++) {
      env_data_buff[chan].push( analogRead( CH_01 + chan));
    }
    for ( int chan = 0; chan < N_CHANS; chan++) {
      env_data_buff[chan].shift();
    }
    delayMicroseconds(SAMPLING_DELAY);
  }

  if (env_data_buff[0].size() != FRAME_LEN) {
    Serial.print("ERROR: Illegal buffer size. Expected: ");
    Serial.print(FRAME_LEN);
    Serial.print(" Got: ");
    Serial.println(env_data_buff[0].size());
  }

  float temp_data = 0;
  float temp_sum[5] = {0};
  float mav_feat[5] = {0};

  for ( int i = 0; i < FRAME_LEN; i++) {
    for ( int chan = 0; chan < N_CHANS; chan++) {
      temp_data = env_data_buff[chan][i] - means[chan];
      temp_sum[chan] += abs(temp_data);
    }
  }

  for ( int chan = 0; chan < N_CHANS; chan++) {
    mav_feat[chan] = temp_sum[chan] * alpha_coeffs[chan];
    mav_feats_buff[chan].push( mav_feat[chan]);
  }

  filterData01(mav_feat);

  float mav_filt_sum = 0;
  for ( int chan = 0; chan < N_CHANS; chan++) {
    mav_filt_sum += filteredData01[chan];
  }

  if (mav_filt_sum < D_THRESHOLD) debounce_flag = 0;

  if ( debounce_flag < 1) {
    if (mav_filt_sum >= THRESHOLD) {

      int feat_idx_count = 0;
      for ( int i = 0; i < FEATURE_LEN; i++) {
        for (int chan = 0; chan < N_CHANS; chan++) {
          features[feat_idx_count] = mav_feats_buff[chan][i];
          feat_idx_count++;
        }
      }

      classification_flag = 1;
      debounce_flag = 1;
    }
  }

  if ( classification_flag > 0) {
    ei_impulse_result_t result = { 0 };

    // the features are stored into flash, and we don't want to load everything into RAM
    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;

    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);

    if (res != 0) return;

    /*Get the prediction with the highest probability*/
    float max_val = 0;
    for ( size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      if (i == 0) {
        max_val = result.classification[i].value;
        max_idx = 0;
      }
      if (result.classification[i].value > max_val) {
        max_val = result.classification[i].value;
        max_idx = i;
      }
      Serial.print(result.classification[i].label);
      Serial.print(" ");
      Serial.println(result.classification[i].value);
    }
    classification_flag = 0;
    hacking_flag = 1;
  }

  if ( hacking_flag > 0) {

    detach_servos(500);

    switch (max_idx) {
      case THUMB_IDX:
        execute_finger_sequence(0);
        break;
      case INDEX_IDX:
        execute_finger_sequence(1);
        break;
      case MIDDLE_IDX:
        execute_finger_sequence(2);
        break;
      case RING_IDX:
        execute_finger_sequence(3);
        break;
      case PINKY_IDX:
        execute_finger_sequence(4);
        break;
      default:
        Serial.println("ERROR DURING MOTOR CONTROL (x_x)");
        break;
    }
    hacking_flag = 0;

    for ( int chan = 0; chan < N_CHANS; chan++) {
      env_data_buff[chan].clear();
    }

    for ( int i = 0; i < FRAME_LEN; i++) {
      for ( int chan = 0; chan < N_CHANS; chan++) {
        env_data_buff[chan].push( analogRead( CH_01 + chan));
      }
      delayMicroseconds(SAMPLING_DELAY);
    }
  }
}


/******************************************************************************
 *                                                                            *
                            FUNCTION DEFINITIONS
 *                                                                            *
 ******************************************************************************/
void compute_alpha_coeffs() {
  for ( int i = 0; i < N_CHANS; i++)
  {
    alpha_coeffs[i] = FRAME_LEN  * norms[i];
    alpha_coeffs[i] = 1.0 / alpha_coeffs[i];
  }
}


void initialize_hand() {
  for (int finger = 0; finger < SERVOS; finger++) moveFinger (finger, OPEN);
  delay (1000);
  for (int finger = 0; finger < SERVOS; finger++) myservo[finger].detach();
  delay (1000);
}


void detach_servos( int time_limit_ms) {
  for ( int i = 0; i < SERVOS; i++) {
    if ( millis() - tmsServo_Attached[i] > time_limit_ms) {
      myservo[i].detach();
    }
  }
}


/* Inference Model --------------------------------------------------------- */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}


/* Motor Control ----------------------------------------------------------- */
void moveFinger(int finger, int pos) {
  if (pos == 1) { // Close
    if (myservo[finger].read() != abs(SRV_CLOSE - SRV_OPEN * servoInv[finger])) { // Close finger
      if (!myservo[finger].attached()) {
        myservo[finger].attach(servoPins[finger], servoMin[finger], servoMax[finger]);
        tmsServo_Attached[finger] = millis();
      }
      myservo[finger].write(abs(SRV_CLOSE - SRV_OPEN * servoInv[finger]));
    }
  }
  else { // Open
    if (myservo[finger].read() != abs(SRV_OPEN - SRV_OPEN * servoInv[finger])) { // Open finger
      if (!myservo[finger].attached()) {
        myservo[finger].attach(servoPins[finger], servoMin[finger], servoMax[finger]);
        tmsServo_Attached[finger] = millis();
      }
      myservo[finger].write(abs(SRV_OPEN - SRV_OPEN * servoInv[finger]));
    }
  }
}


void execute_finger_sequence( int finger_idx) {
  char *finger_labels[] = {"THUMB", "INDEX", "MIDDLE", "RING", "PINKY"};
  int period_ms = 1000;

  moveFinger (finger_idx, CLOSE);
  Serial.print("Flexing: ");
  Serial.println(finger_labels[finger_idx]);
  delay(period_ms);
  moveFinger (finger_idx, OPEN);
  Serial.print("Relaxing: ");
  Serial.println(finger_labels[finger_idx]);
  delay(period_ms);
}

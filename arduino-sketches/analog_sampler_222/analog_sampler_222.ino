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
#define FEATURE_LEN 11
#define SIGNAL_LEN 80
#define STEP_LEN 6
#define THRESHOLD 7.15
#define D_THRESHOLD 7.05

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
int servoMin[SERVOS] = {1000,544,544,544,544}; // minimum pwm value for each servo
int servoMax[SERVOS] = {1750 ,2400,2400,2400,2400}; // maximum pwm value for each servo
int servoInv[SERVOS] = {0,0,0,0,0}; // indicates if servo direction is inverted

unsigned long tmsServo_Attached[SERVOS]; // time since last servo attach. Used to reduce power comsumption
unsigned long previousTime = 0;

Servo myservo[SERVOS];
/* ------------------------------------------------------------------------- */

/* Circular Buffer ---------------------------------------------------- */
//unsigned long analog_read_time;
//
//CircularBuffer<float, BUFFER_SIZE> buffer_ch01;
//CircularBuffer<float, BUFFER_SIZE> buffer_ch02;
//CircularBuffer<float, BUFFER_SIZE> buffer_ch03;
//CircularBuffer<float, BUFFER_SIZE> buffer_ch04;
//CircularBuffer<float, BUFFER_SIZE> buffer_ch05;
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
//auto sampling_timer = timer_create_default();
//int debug_counter = 0;
uint32_t temp_data;
/* ------------------------------------------------------------------------- */

/* Inference Model --------------------------------------------------------- */
// static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
float features[] = {
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
// bool debug_timer(void *){
//   Serial.println("ANALOG SAMPLING: START");
//   Serial.print("\tData: ");
//   for (int chan=CH_01; chan <= CH_05; chan++){
//     temp_data = analogRead(chan);
//     Serial.print(temp_data);
//     if( chan != CH_05) Serial.print(", ");
//   }
//   Serial.println();
//   Serial.println("ANALOG SAMPLING: DONE");  
//   return true;
// }
/* ------------------------------------------------------------------------- */

/* Filtering --------------------------------------------------------------- */
//filter formula can be found here:
// https://www.earlevel.com/main/2003/02/28/biquads/
// y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2] – b1*y[n-1] – b2*y[n-2]
//b0, b1, b2, a1, a2 can be calculated here:
//https://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/
float a0 = 0.010995248654511658;
float a1 = 0.021990497309023315;
float a2 = 0.010995248654511658;
float b1 = -1.6822395838277024;
float b2 = 0.7262205784457494;

float x[N_CHANS][3];
float y[N_CHANS][2];
float filteredData01[5];

// call this on each new sample
void filterData01(float newValue[])
{
  for (int j = 0; j < N_CHANS; j++)
  {
    x[j][2] = x[j][1];
    x[j][1] = x[j][0];
    x[j][0] = newValue[j];
    filteredData01[j] = a0*x[j][0]
                      + a1*x[j][1]
                      + a2*x[j][2]
                      - b1*y[j][0]
                      - b2*y[j][1];
    y[j][1] = y[j][0];
    y[j][0] = filteredData01[j];
  }
  return;
}

//call this in setup
float initFilter01()
{
  for (int j=0; j< N_CHANS; j++)
  {
    x[j][0] = 0;
    x[j][1] = 0;
    x[j][2] = 0;
    y[j][0] = 0;
    y[j][1] = 0;
  }  
}

float x_sum[3];
float y_sum[2];
float filteredData02;

void filterData02(float newValue)
{
    x_sum[2] = x_sum[1];
    x_sum[1] = x_sum[0];
    x_sum[0] = newValue;
    filteredData02 = a0*x_sum[0]
                    + a1*x_sum[1]
                    + a2*x_sum[2]
                    - b1*y_sum[0]
                    - b2*y_sum[1];
    y_sum[1] = y_sum[0];
    y_sum[0] = filteredData02;
    return;
}

float x_trig[N_CHANS][3];
float y_trig[N_CHANS][2];
float filteredData03[5];

// call this on each new sample
void filterData03(float newValue[])
{
  for ( int j = 0; j < N_CHANS; j++)
  {
    x_trig[j][2] = x_trig[j][1];
    x_trig[j][1] = x_trig[j][0];
    x_trig[j][0] = newValue[j];
    filteredData03[j] = a0*x_trig[j][0]
                      + a1*x_trig[j][1]
                      + a2*x_trig[j][2]
                      - b1*y_trig[j][0]
                      - b2*y_trig[j][1];
    y_trig[j][1] = y_trig[j][0];
    y_trig[j][0] = filteredData03[j];
  }
  return;
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
void ei_printf(const char *format, ...) 
{
   static char print_buf[1024] = { 0 };

   va_list args;
   va_start(args, format);
   int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
   va_end(args);

   if (r > 0) 
   {
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
      if (!myservo[finger].attached()) 
      {
        myservo[finger].attach(servoPins[finger], servoMin[finger], servoMax[finger]);
        tmsServo_Attached[finger] = millis();
      }
      myservo[finger].write(abs(SRV_OPEN - SRV_OPEN*servoInv[finger]));
    }
  }
}

void execute_finger_sequence( int finger_idx)
{
  int period_ms = 1000;
  moveFinger (finger_idx, CLOSE);
  Serial.println("\t\tHAND STATUS: CLOSED");
  delay(period_ms);
  moveFinger (finger_idx, OPEN);
  Serial.println("\t\tHAND STATUS: OPEN");
  delay(period_ms);
  return;
}
/* ------------------------------------------------------------------------- */

/*****************************************************************************/
/*****************************************************************************/
/************************************ SETUP **********************************/
/*****************************************************************************/
/*****************************************************************************/
#define SAMPLING_PIN 3
#define DELAY_PIN 4
void setup() 
{
  /* ----------------------------------------------------------------------- */
  Serial.begin( BAUD_RATE);
  /* ----------------------------------------------------------------------- */
  for (int finger = 0; finger < SERVOS; finger++) moveFinger (finger, OPEN);  // Open Hand
  delay (1000);                                            // Wait for servos
  for (int finger = 0; finger < SERVOS; finger++) myservo[finger].detach();   // Relax servos
  delay (1000);
  /* ----------------------------------------------------------------------- */
  for( int i = 0; i < N_CHANS; i++)
  {
    alpha[i] = FRAME_LEN * stdevs[i] * norms[i];
    alpha[i] = 1.0 / alpha[i];
  }
  // analog_read_time = millis();  //Initialize reference timestamp
  /* ----------------------------------------------------------------------- */
  pinMode(SAMPLING_PIN, OUTPUT);
  pinMode(DELAY_PIN, OUTPUT);
}

/******************************************************************************/
/******************************************************************************/
/************************************ LOOP ************************************/
/******************************************************************************/
/******************************************************************************/
size_t max_idx = 0;

int classification_flag = 0;
int hacking_flag = 0;
int debounce_flag = 0;

void loop() 
{  
  float mav[5]={0,0,0,0,0};
  float temp_data = 0;

  for (int i = 0; i < FRAME_LEN; i++)
  {
    digitalWrite(SAMPLING_PIN, HIGH);
    for(int j = 0; j < N_CHANS; j++)
    {
      temp_data = analogRead(CH_01+j);
      temp_data = temp_data - means[j];
      temp_data = abs(temp_data);
      mav[j]+= temp_data;
      if( i == ( FRAME_LEN -1))
      {
        mav[j] *= alpha[j];
      }
    }
    if( i < FRAME_LEN-1){
      delay(4);
    }
  }

  filterData01(mav);

  float mav_sum = 0;
    
  for( int j = 0; j < N_CHANS; j++)
  {
    mav_sum += filteredData01[j]; 
  }

  filterData02( mav_sum);
  Serial.print( filteredData02);
  Serial.print( " , ");

  float env_data[SIGNAL_LEN][N_CHANS]= {};
  float mav_feat[5]={};
  int feat_idx_count = 0;

  if( filteredData02 < D_THRESHOLD) debounce_flag = 0;
  if( debounce_flag < 1)
  {
    if( filteredData02 > THRESHOLD)
    {
      for ( int i = 0; i < SIGNAL_LEN; i++)
      {
        for( int j = 0; j < N_CHANS; j++)
        {
          env_data[i][j] = analogRead( CH_01+j);
        }
        delay(4);
      }
      
      for( int i = 0; i < FEATURE_LEN; i++)
      {
        for ( int k = i * STEP_LEN; k < i * STEP_LEN + FRAME_LEN; k++)
        {
          for (int j = 0; j < N_CHANS; j++)
          {
            temp_data = env_data[k][j];
            temp_data = temp_data - means[j];
            temp_data = abs( temp_data);
            mav_feat[j] += temp_data;
            if(k == FRAME_LEN -1)
            {
              mav_feat[j] += alpha[j] * mav_feat[j];
            }
          }
        }
        
        filterData03(mav_feat);
        
        for(int j = 0; j<N_CHANS;j++)
        {
          features[feat_idx_count] = filteredData03[j];
          feat_idx_count++;
          filteredData03[j] = 0; 
        }
      }
      
      for ( int i = 0; i < FEATURE_LEN; i++)
      {
        Serial.print(features[i]);
        Serial.print(" ");
      }
      
      Serial.println();
      classification_flag = 1;
      debounce_flag = 1;
    }
    
    else
    {
      Serial.print(6);
    }
  /***************************************************/  
    Serial.println();
  }
  
  if( classification_flag > 0)
  { 
    ei_impulse_result_t result = { 0 };
    
    // the features are stored into flash, and we don't want to load everything into RAM
    signal_t features_signal;
    features_signal.total_length = sizeof(features) / sizeof(features[0]);
    features_signal.get_data = &raw_feature_get_data;

    Serial.println("\tCLASSIFICATION: START");
    
    // invoke the impulse
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);

    if (res != 0) return;

    Serial.println("\tCLASSIFICATION: END");  
    
    /* ------------------------------------------------------------------------- */
    /*Get the prediction with the highest probability*/
    Serial.println("\tCLASS SELECTION: START");
    float max_val = 0;
    for( size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++)
    {
      if (i==0)
      {
        max_val = result.classification[i].value;
        max_idx = 0;
      }
      if(result.classification[i].value > max_val)
      {
        max_val = result.classification[i].value;
        max_idx = i;
      }
    }
    Serial.print("\t\tPREDICTED CLASS: ");
    Serial.println(result.classification[max_idx].label);
    Serial.print("\t\tCONFIDENCE: ");
    Serial.println(max_val);
    Serial.print("\t\tINDEX: ");
    Serial.println(max_idx);
    Serial.println("\tCLASS SELECTION: END");
    classification_flag = 0;
    hacking_flag = 1;
  }
  if ( hacking_flag > 0)
  {
    /* ------------------------------------------------------------------------- */
    // Detach servo if time since last attach exceeds 500 ms.
    for ( int i = 0; i < SERVOS; i++)
    {
      if ( millis() - tmsServo_Attached[i] > 500) 
      {
        myservo[i].detach();
      }
    }

    Serial.println("\tMOTOR CONTROL: START");
    switch(max_idx)
    {
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
    Serial.println("\tMOTOR CONTROL: END");
    Serial.println("<--- THE LOOP ENDS HERE --->");
    hacking_flag = 0;
  }
}

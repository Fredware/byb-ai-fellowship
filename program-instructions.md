# Hacker Hand: 

## How To Use It

### Scripts

There are three main scripts for getting the hacker hand to work:

- `data_acquisition.ino`: Sample data from the SpikerShield Pro<sup>TM</sup> at a specified rate and pass it to `data_formatter.py` by sending data packets over the Serial Port.
- `data_formatter.py`: Receive packets of data over the Serial Port and put them into `.csv` files formatted according to Edge Impulse Studio requirements.
- `real_time_control.ino`: Sample data from the SpikerShield Pro<sup>TM</sup> at a specified rate and pass it to a trained model downloaded from Edge Impulse Studio. Use the output of the model to move one finger at a time.

There is an additional script I used early on for data visualization and analysis. You don't need this for the hand to work, but it might help you understand the rationale behind the pre-processing pipeline implemented in `data_acquisition.ino` and `real_time_control.ino`.

- `data_visualizer.ino`: Take a session of data recorded using the Spike Recorder Software,  find all the instances of an EMG event, visualize how the average activity of a channel looks like for all gestures.

### Data Acquisition

1. Connect the sensors on your forearm following the guidelines specified in the [**Atlas of Muscle Innervation Zones**](https://www.springer.com/gp/book/9788847024625). I targeted the following muscles:

   1. Palmaris Longus
   2. Flexor Carpi Radialis
   3. Brachioradialis
   4. Extensor Carpi Radialis
   5. Abductor Digiti Minimi

2. In the data acquisition script setup the following macros according to your own unique setup

   1. `SAMPLING_FREQ`
   2. `SAMPLING_DELAY`
   3. `N_CHANS`
   4. `CH_01` - `CH_05`
      1. Note that the program relies on the assumption that these pins are consecutive.

3.  Change the following macros based on this graph:

   ![How do i compute the number of overlapping frames an given audio file has?  - Mathematics Stack Exchange](https://i.stack.imgur.com/d7Rdx.png)

   1. `FRAME_LEN`: number of samples that make a frame
   2. `STEP_LEN`: number of samples that make a step

4. Set the value of this macro so that it matches your neural network architecture.

   1. `FEATURE_LEN`: The number of feature per channel that you plan to use to classify the gesture
   2. `FEATURE_BYTESIZE`: The byte size of a single feature element according to the Arduino IDE. e.g. I used an array of floats, for my feature array, therefore, according to the Arduino Docs, a float occupies 4 bytes in memory.

5. If you choose to use the double threshold technique for detecting EMG onsets, change both of the following macros

   1. `THREHOLD`: Anything above this value will be considered an EMG event that needs to be classified
   2. `D_THRESHOLD`: Prevents new EMG events from begin sent to the classifier until the signal goes below this threshold.

   For more info see [Schmitt trigger](https://en.wikipedia.org/wiki/Schmitt_trigger)

6. Set the constant variable `means` by taking the average of the channel readings when the subject is not performing any gestures.

7. Set the constant variable `norms` by takin the maximum values for each channel after a subject has performed an MVC for each gesture.

8. Compile, upload and run the Arduino sketch. Then, modify the appropriate fields in `data_formatter.py` and run it in parallel to `data_acquisition.ino`.

9. Each time your cumulative channel activity crosses `THRESHOLD`, `data_acquisition.ino` will send a packet of data containing the last 11 MAV features preceding the crossing event. Each individual packet of data representing an event will then save it as a `.csv` ready to be uploaded to Edge Impulse Studio.

### Real-Time Control

Once you have trained a neural network, make sure to update the macros specified in the previous section, and then modify the script to connect each classification outcome to the finger motion you want to execute. Then, connect the servos to the pins specified in the global variable `servoPins` and watch the hand respond to your gestures.
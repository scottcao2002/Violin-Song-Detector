# Violin-Song-Detector
This is the code for a device that can be trained to identify 3 different violin songs using an STMicroelectronics SensorTile.

# Demonstartion
A demonstration of the device can be found with the following link: https://www.youtube.com/watch?v=F_4bsrlunVo

# User Instructions
The SensorTile should be attached to the right arm of a violin player in such a way that the x-axis of the SensorTile is perpendicular to an imaginary line drawn from the shoulder to the wrist. This way, the x-axis is the axis that will change in angle the most while the song is being played. The following steps are then taken:

1. The player assumes rest position. Since the SensorTile obtains the arm angles relative to some reference angle, rest position will serve as the reference angle in this project. Rest position was chosen because it is universal to all orchestral violinists.

2. The player is prompted to get into initial playing position. The user has 5 seconds to go from rest position to putting the bow on the string. The player is not to play at this time.

3. The player is prompted to begin the song. The user must play for at least 10 seconds, and once that is done, the player will assume rest position once the song is finished to indicate to the SensorTile that the song is finished.

During the training stage, those 3 steps are done 3 times for 3 different songs to obtain the data for those songs. Then, the SensorTile will train with the input data using a neural network. Once done, the player may go into the testing stage. In this stage, the player will be prompted to go through those 3 steps again, and once the player plays any of the 3 songs, the STM will be able to correctly identify which song it is by the end.

# How it works
The STM uses a neural network in order to train to identify the 3 different songs. There are 3 inputs for the neural network: the initial arm angle of the violin player right before the song starts, the intermediate arm angle of the violin player 10 seconds into the song, and the length of the song in seconds. There are 9 intermediate neurons (the hidden layer), which are then linked to 3 output neurons, corresponding to the 3 different songs.

# Main.c
This is the main program that the STM will run. It contains the following useful functions:

motion_softmax:

This function is used to normalize the 3 input neurons so one input cannot dominate over the others. Since the maximum arm angle is estimated to be 110 degrees (through experimentation), the first 2 neurons, which correspond to arm angles, are divided by 110. Since the maximum song length is about 50 seconds, the value of the 3rd neuron is divided by 50.

getAngularVelocity:

This function obtains the angular velocity of the SensorTile in the x, y, and z axes, which is useful in order to obtain the angle of the SensorTile at some given time. This is done by taking advantage of the calculus trapezoid rule. If the angular velocity at some time step is added to the angular velocity at the time step before, divided by 2, and multiplied by the time for the sample period, the change in the angle of the SensorTile is obtained. If that change is added to the angle at the time step before, this obtains the angle at the new time step.

Feature_Extraction_Gyro:

This function obtains the values for the 3 input neurons. After initializing all the necessary variables like the sample time period, it obtains the arm angle of the violin player at rest position. This will be the reference angle. Then, the user is prompted to get in playing position and the first while loop starts. The while loop will continuously update the arm angle for 5 seconds before it breaks. In these 5 seconds, the player should have gone from rest position to the initial playing position. The arm angle at this time is set as the first neuron. Then, the user is prompted to begin playing, and the second while loop starts. Just like the first, it runs for 10 seconds before it breaks, and at the end of the 10 seconds, it stores the arm angle as the second neuron. Finally, the third while loop starts, which is an infinite loop that will only break once the arm angle is less than 5 degrees. This is because when the angle is less than 5 degrees, the player is determined to have assumed rest position again. The time from the beginning of the song to that point is stored as the 3rd neuron.

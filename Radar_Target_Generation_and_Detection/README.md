# Radar_Target_Generation_and_Detection

## 1. Variables and Simple Equations

- Variables from environment and target variables;
- Equations: Bandwidth, Chirp time and Slope.
![img1](./img/part1.png?raw=true)

## 2. FMCW

- Setting parameters to design FMCW waveform;
- Creating Transmitter, Receiver and beat signal;
- The same size of vectors were defined to range_covered and time_delay.
![img2](./img/part2.png?raw=true)

## 3. Target Simulation

![img3](./img/part3.png?raw=true)

## 4. Range Measurement (FFT1)

- Reshape to (Nr, Nd) -> Define size of range and Doppler Fast Fourier Transform (FFT);
- FFT ran in Mix along Nr;
- Double-sided signal. Transformation to absolute.
- Plot configurations.
![img4](./img/part4.png?raw=true)

## 5. Plot 1 (FFT1)

*Note: The background color and font color were changed in MATLAB Plot Tool.
![img5](./img/part5.png?raw=true)

## 6. Range Doppler response (FFT2)

- Map Generation;
- 2D-FFT of Mix (beat signal);
- Axis convertion: from bin sizes to range and Doppler;
- Double-sided signal. Transformation to absolute;
- Plot configurations (using surf function);
![img6](./img/part6.png?raw=true)

## 7. Plot 2 (FFT2)

*Note: The background color and font color were changed in MATLAB Plot Tool.
![img7](./img/part7.png?raw=true)

## 8. Training Cells and Guard Cells (CFAR)

- First step: Implementation of Training Cells and Guard Cells in both dimensions + Offset. (***Important***)
![img8](./img/part8.png?raw=true)

## 9. Implementation (CFAR)

- noise_level and n_cell to define threshold after (threshold = noise_level / n_cell);
- Store noise_level to each iteration on training cells;
- Loop Cells Under Test (CUT) across Range Doppler Map (RDM);
- Sum the signal level with all the training cells, in every iteration;
- Conver from logarithmic to linear to execute sum (db2pow);
- Average all values in sum for all training cells used, and then convert back (from linear to logarithmic using pow2db);
- Add offset to determine threshold;
- If CUT > threshold, RDM = 1, else 0;
- It will define a threshold block, smaller than RDM (CUT cannot be located at the edges due to that);
- Just a small amount of cells will not be thresholded, and will be equal to 1.
![img9](./img/part9.png?raw=true)

- Plot configurations (using surf function).
![img10](./img/part10.png?raw=true)

## 10. Plot 3 (CFAR)

*Note: The background color and font color were changed in MATLAB Plot Tool.
![img11](./img/part11.png?raw=true)


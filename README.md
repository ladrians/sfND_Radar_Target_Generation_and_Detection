# Sensor Fusion Nanodegree

## Radar Target Generation and Detection
Luciano Silveira
July, 2020

### Specification

![sample](./images/p04_01_01.png)

## Basic Build Instructions

1. Clone this repo.
2. Create an account in Download [MATLAB](https://www.mathworks.com/mwaccount/register).
3. Download MATLAB on your computer.
4. Run [radar-target-generation-and-detection.m](radar-target-generation-and-detection.m).

### FMCW Configuration

Using the given system requirements, design a FMCW waveform. Find its Bandwidth (B), chirp time (Tchirp) and slope of the chirp.

For given system requirements the calculated slope should be around 2e13

### Moving Target Generation

Simulate Target movement and calculate the beat or mixed signal for every timestamp.

A beat signal should be generated such that once range FFT implemented, it gives the correct range i.e the initial position of target assigned with an error margin of +/- 10 meters.

### Signal Propagation

Implement the Range FFT on the Beat or Mixed Signal and plot the result.

A correct implementation should generate a peak at the correct range, i.e the initial position of target assigned with an error margin of +/- 10 meters.

### Processing Received Reflected Signal

### Range/Doppler FFT

#### CFAR Detection

Implement the 2D CFAR process on the output of 2D FFT operation, i.e the Range Doppler Map.

The 2D CFAR processing should be able to suppress the noise and separate the target signal. The output should match the image shared in walkthrough.

 * Implementation steps for the 2D CFAR process.
 * Selection of Training, Guard cells and offset.
 * Steps taken to suppress the non-thresholded cells at the edges.

### Links

 * [Project Rubric](https://review.udacity.com/#!/rubrics/2548/view)

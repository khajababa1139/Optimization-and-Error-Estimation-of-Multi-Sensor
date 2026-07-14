
# Hover Data Acquisition

This folder contains the uncalibrated hover-flight data acquisition run for the Crazyflie 2.1 platform.

## Purpose

The goal of this experiment is to record the Crazyflie state during a steady hover and save the flight log as a CSV file for later linear and radial analysis.

## Experiment Summary

- The estimator is reset before flight.
- The vehicle is allowed to warm up for 30 seconds while stationary.
- Recording starts before arming.
- The drone takes off to 1.5 m, holds position for the hover duration, and then lands smoothly.
- The script stores position and attitude data at 100 ms intervals.

## Files Generated

- A timestamped CSV file named like `hover_YYYY-MM-DD_HH-MM-SS.csv`.
- Plots for x, y, z, roll, pitch, and yaw are shown after the flight.

## Script

The acquisition process is implemented in `hover_data_acq.py`.

## Usage

Run the script from this folder after connecting the Crazyflie and configuring the radio URI if needed:

```bash
python3 hover_data_acq.py
```

## Notes

- This is an uncalibrated iteration.
- The initial position is written as the first row of the CSV at time zero.
- Landing is not recorded in the output file.

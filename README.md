<<<<<<< HEAD
# RPLidar operations files are included here.
=======
# Lidar Logger

This project contains two applications:

1. Data Logger: Records raw lidar data to CSV files
2. Visual Logger: Real-time visualization of lidar data

## Building

Run 'make' in the root directory to build both applications.

## Usage

### Data Logger
Run from the app/data_logger directory:
./data_logger --channel --serial /dev/ttyUSB0 1000000

### Visual Logger
Run from the app/visual_logger directory:
./visual_logger --channel --serial /dev/ttyUSB0 1000000

>>>>>>> 74002a2 (first commit)

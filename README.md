# ForRocket
ForRocket calculate 6 degree-of-freedom rocket trajectory.

## Overview
Calculate trajectory of non-guidance/non-controlled single-stage rocket without attitude controll. Assume single-port fuel hybrid rocket and automatically calculate center of gravity. Trajectory is limited to three-dimentional plane(East-North-Up), flight of rocket of orbit can not be calculated.(Therefore, trajectory with long downrange has a large error)

This tool works with Python3.x and is environment independent. That depends on the numpy, scipy, pandas, matplotlib, simplekml, tqdm libralies.

## Example
|Position|Velocity|
|---|---|
|![positionenu](https://user-images.githubusercontent.com/8069773/41517065-56373a9c-72f3-11e8-85d5-cc4122720e3a.png)|![velocityenu](https://user-images.githubusercontent.com/8069773/41517075-5ec56eae-72f3-11e8-875a-0b7e114df4f6.png)|

## Usage
1. Make configuration json file.(Based rocket_config.json and EngineConfig.json)
2. Make thrust file, drag curve etc.(optional. Fixed value can be specified with json file.)
3. Script execution.
```
python ForRocket.py rocket_config.json 1
```
* 1st argument: config json file name
* 2nd argument: run mode number(default 1, 1: single trajectory, 2: table for wind)

4. Calculation result is stored in result directory.

## License
ForRocket is released under the MIT License.

# ForRocket
ForRocket calculate 6 degree-of-freedom rocket trajectory.

## Overview
Calculate trajectory of non-guidance/non-controlled single-stage rocket without attitude controll. Assume single-port fuel hybrid rocket and automatically calculate center of gravity. 

This tool works with Python3.x and is environment independent. That depends on the numpy, scipy, pandas, matplotlib, simplekml, tqdm, [pymap3d(ver.=1.7.10.1)](https://pypi.org/project/pymap3d/1.7.10.1/) libralies.

## Usage
1. Make configuration json file.(Based rocket_config.json and EngineConfig.json)
2. Make thrust file, drag curve etc.(optional. Fixed value can be specified with json file.)
3. Script execution.
```
python ForRocket.py rocket_config.json -[f/m]
```
* 1st argument: config json file name
* 2nd argument: output mode (default None, f: full output, None:dafault output ,m: minimum output)

4. Calculation result is stored in result directory.

## License
ForRocket is released under the MIT License.

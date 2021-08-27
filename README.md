# ForRocket - 6DoF rocket trajectory solver
<img src="https://user-images.githubusercontent.com/8069773/76425916-a54e6a00-63ed-11ea-8cc0-0d63b9b2cf9a.png" width="400px">

## Feature
* Calculate 6 degree-of-freedom rocket trajectory.
* Liquid, Solid and Hybrid ... no fixed type of rocket engine.
* Configurable sequence of event. Cutoff, Separation, despin, jettson, etc...
* Easy input by json file and easy post-process by csv file.
* User easily extend by using input json and output csv.
* Prvide only trajectory solver. Satisfy extend tool.

## Testing Feature
* Attitude control flight
* Multi-stage flight

## Getting Started
### Installation
```sh
$ git clone https://github.com/sus304/ForRocket.git
$ cd ForRocket
$ ./setup_libs.sh
$ mkdir build && cd build
$ cmake ..
$ make
$ cd ..
```

### Sample Execute
```sh
$ cp build/ForRocket.exe examples/
$ cd examples
$ ./ForRocket.exe sample_config_solver.json
```

### Input file
* solver_config.json - launch configuration and stages info
* stage_config_list.json - config json file list at stage
  * sequence_of_event.json - SOE configuration at stage
  * rocket_config.json - structure and aerodynamics configuration at stage
  * engine_config.json - engine configuration at stage
  * extra csv file - time vs thrust, mach vs CA, etc...

## Library
* [boost](https://www.boost.org/)
* [Eigen](http://eigen.tuxfamily.org/)
* [json @nlohman](https://github.com/nlohmann/json)


## License
ForRocket is released under the [MIT License](http://opensource.org/licenses/MIT).

Copyright &copy; 2020- Susumu Tanaka
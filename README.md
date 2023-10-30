## Project name: sai2-simulation (version: 2.0.0)
This project uses Semantic Versioning (http://semver.org/).

### Project short description:
The Sai2-Simulation library implements a complete multi-articulated body 
simulation system with the following components:

* Articulated body specification
* Time-stepping integrator
* Multi-point contact detection
* Multi-point multi-body collision resolution
* Multi-point multi-body contact resolution (with Coulomb friction)

### SAI library dependencies:
Chai3d: https://github.com/manips-sai-org/chai3d
sai2-urdfreader : https://github.com/manips-sai-org/sai2-urdfreader
sai2-common : https://github.com/manips-sai-org/sai2-common

### Other dependencies
freeglut3, libxmu, libstdc++.

### Installation instructions:
```
mkdir build && cd build
cmake .. && make -j4
```

### Uninstallation instructions: 
```
rm -r build
rm -r ~/.cmake/packages/SAI2-SIMULATION
```

### Getting started:
Take a look at sample applications under examples/.
You can run them from build/examples/.

### License:
Currently pending licensing. PLEASE DO NOT DISTRIBUTE.

### Project contributors:

* Kyong-sok Chang
* Diego Ruspini
* Luis Sentis
* Jaehong Park
* Francois Conti
* Shameek Ganguly
* Mikael Jorda

### For questions, contact:
manips.sai@gmail.com  
mjorda@stanford.edu  

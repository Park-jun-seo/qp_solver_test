# qp_tutorial

qp tutorial for ros2

# Eigen Installation

official documents: https://eigen.tuxfamily.org/dox-3.3/group__TutorialMatrixArithmetic.html

```
sudo apt-get install libeigen3-dev
```

The default path is: `/usr/include/eigen3`. You should use

```
#include <eigen3/Eigen/Dense>
```

# qpOASES Installation
official repository: https://github.com/coin-or/qpOASES

```
git clone https://github.com/coin-or/qpOASES
cd qpOASES
mkdir build
cd build
cmake ..
sudo make 
sudo make install
source ~/.bashrc
```
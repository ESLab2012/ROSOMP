# Overview
This repository presents the implementation of **ROSOMP**, a framework that integrates OpenMP-based workload management directly within the ROS 2 executor. 
By reusing executor threads and introducing unified scheduling, ROSOMP optimizes resource utilization and ensures real-time performance. 
We developed the OMP Scheduler using the GCC 15.0.0 and LLVM/Clang 21.0, both of which supporting OpenMP 5.1 runtime, and implemented the OMP Arbiter on the ROS 2 [Humble](https://docs.ros.org/en/humble/index.html) release.

# Getting Started

## Description
This repository contains two fundamental parts, and each part consists of two folders:
<ul>
<li> The `libgomp` or `libomp` folder implements the OMP Scheduler by leveraging the OpenMP runtime support provided by the GCC or LLVM/Clang compilers.</li>
<li> There are three ROS 2 packages involved in our ROSOMP: 
The `rcl` package and the `rclcpp` package for implementing the OMP Arbiter, and `demo` code for testing the real-time performance of ROSOMP.</li>
</ul>

## Prerequisites
ROSOMP is based on the latest version of ROS 2 Humble. Before compiling and building ROSOMP, you need to install ROS 2 on your Ubuntu system. Follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) for detailed instructions.

### Build

#### GCC Source Compilation and Replacement

##### 1. Install Dependencies.
```bash
sudo apt-get install build-essential libgmp-dev libmpfr-dev libmpc-dev flex bison
```

##### 2. Download GCC Source Code.
You can modify the URL to download a specific version of the GCC source code.

```bash
git clone https://gcc.gnu.org/git/gcc.git
```

##### 3. Compile GCC Source Code.

###### Create a build folder
```bash
mkdir build && cd build
```

###### Configure build options
```bash
# You may modify the `--prefix` option to configure the installation path of GCC.
../configure --prefix=/usr/local/gcc --enable-languages=c,c++ --disable-multilib
```

###### Compile GCC
```bash
# The `-j$(nproc)` option automatically detects the number of available CPU cores for parallel compilation.
make -j$(nproc)
```

###### Install GCC
```bash
# To install GCC, superuser privileges are required.
sudo make install
```

###### Update environment variables
```bash
# Modify the paths accordingly if a different installation folder of GCC is chosen.
export PATH=/usr/local/gcc/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/gcc/lib64:$LD_LIBRARY_PATH
```

###### Test if GCC has been successfully replaced
```bash
which gcc
# Expected output: /usr/local/gcc/bin/gcc
```

##### 4. Replace GCC Source Code.

(1) Use the files from the `libgomp/` folder, namely `libgomp.map`, `omp.h.in` and `parallel.c`, to replace the corresponding original files in the `GCC/libgomp/` folder.

(2) After replacing the necessary files, recompile the GCC source code by repeating the compilation steps.

#### LLVM Source Compilation and Replacement

Instructions for downloading and building LLVM from source code can be found at [here](https://llvm.org/docs/GettingStarted.html).

```bash
# Installation and update of Cmake
sudo snap install cmake --classic
export PATH=/snap/bin:$PATH
cmake --version

# Download LLVM source code
git clone https://github.com/llvm/llvm-project.git
cd llvm-project

mkdir build && cd build
# You can modify the `-DLLVM_TARGETS_TO_BUILD` option to select build targets, and use `-DCMAKE_INSTALL_PREFIX` option to configure the LLVM installation path.
cmake -G Ninja ../llvm \
  -DLLVM_ENABLE_PROJECTS="clang;openmp" \
  -DCMAKE_BUILD_TYPE=Release \
  -DLLVM_TARGETS_TO_BUILD="X86" \
  -DCMAKE_INSTALL_PREFIX=~/llvm-install
# Compile and install LLVM
ninja
ninja install

# update environment variables
export LD_LIBRARY_PATH=/llvm-install/lib/x86_64-unknown-linux-gnu:$LD_LIBRARY_PATH
export CC=/llvm-install/bin/clang
export CXX=/llvm-install/bin/clang++
```

Replace the original files in `LLVM-project/openmp/runtime/src/` with the ones from the `libomp` folder, then recompile the LLVM source code.

#### ROS 2 Source Compilation

##### 1. Create a ROS 2 Workspace.

Create a ROS 2 workspace named `ROSOMP`.
Detailed instructions can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).
Then, download the `src` folder from this repository into the ROSOMP workspace. 

##### 2. Replace GCC or LLVM Installation Paths in Source Files.

Modify the following seven files in the `/ROSOMP/src/` folder to replace the original GCC or LLVM installation paths. 
Each original GCC or LLVM installation path is marked with the text "Replace with the correct path".

```bash
src/intra_process_demo/include/node_common.hpp
src/intra_process_demo/CMakeLists.txt
src/rcl/src/rcl/wait.c
src/rcl/CMakeLists.txt
src/rclcpp/src/rclcpp/executors/multi_threaded_executor.cpp
src/rclcpp/src/rclcpp/executor.cpp
src/rclcpp/CMakeLists.txt
```

##### 3. Build the ROS 2 Workspace.

```bash
# Build the ROS 2 workspace with GCC
# Source the ROS 2 setup script.
. ~/ros2_humble/install/local_setup.sh
# Update the environment variables.
export PATH=/usr/local/gcc/bin:$PATH 
export LD_LIBRARY_PATH=/usr/local/gcc/lib64:$LD_LIBRARY_PATH 
# Build the workspace.
colcon build --allow-overriding rcl rclcpp
# Source the local setup script after building.
source install/local_setup.sh

# Build the ROS 2 workspace with LLVM
. ~/ros2_humble/install/local_setup.sh
export PATH=/snap/bin:$PATH
export LD_LIBRARY_PATH=/llvm-install/lib/x86_64-unknown-linux-gnu:$LD_LIBRARY_PATH
export CC=/llvm-install/bin/clang
export CXX=/llvm-install/bin/clang++
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/lib/x86_64-linux-gnu/cmake/spdlog
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/lib/x86_64-linux-gnu/cmake/opencv4
colcon build --allow-overriding rcl rclcpp --cmake-force-configure --cmake-args -DCMAKE_EXE_LINKER_FLAGS="-ldl"
source install/setup.sh
```

##### 4. Run the Given Example.

```bash
# If you build the ROS 2 workspace with LLVM, you can use the following instruction to set the spin-wait delay to zero.
export KMP_BLOCKTIME = 0

ros2 run intra_process_demo two_node_pipeline
# Alternatively, there are another case study version
ros2 run intra_process_demo two_node_pipeline_casestudy
```
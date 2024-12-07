# Overview
This repository presents the implementation of **ROSOMP**, a framework that integrates OpenMP-based workload management directly within the ROS 2 executor. 
By reusing executor threads and introducing unified scheduling, ROSOMP optimizes resource utilization and ensures real-time performance. 
We developed the OMP Scheduler using the GCC 15.0.0 compiler with OpenMP 5.1 runtime support and implemented the ROSOMP Executor and OMP Orchestrator on the ROS 2 [Humble](https://docs.ros.org/en/humble/index.html) release.

# Getting Started

## Description
This repository contains two fundamental parts:
<ul>
<li>The libgomp module implements the OMP Scheduler based on the GCC compiler's OpenMP runtime support.</li>
<li>
There are three ROS 2 packages involved in our ROSOMP. 
The `rcl` package and the `rclcpp` package for implementing the ROSOMP Executor and OMP Orchestrator, and `demo` code for testing the real-time performance of ROSOMP.</li>
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
###### (1) Create a build directory.
```bash
mkdir build && cd build
```
###### (2) Configure build options.
You may modify the `--prefix` option to configure the installation path of GCC.
```bash
../configure --prefix=/usr/local/gcc --enable-languages=c,c++ --disable-multilib
```

###### (3) Compile GCC.
The `-j$(nproc)` option automatically detects the number of available CPU cores for parallel compilation.
```bash
make -j$(nproc)
```

###### (4) Install GCC.
To install GCC, superuser privileges are required.
```bash
sudo make install
```

###### (5) Update environment variables.
Modify the paths accordingly if a different installation directory of GCC is chosen.
```bash
export PATH=/usr/local/gcc/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/gcc/lib64:$LD_LIBRARY_PATH
```

###### (6) Test if GCC has been successfully replaced
```bash
which gcc
# Expected output: /usr/local/gcc/bin/gcc
```
# Overview
This repository presents the implementation of **ROSOMP**, a framework that integrates OpenMP-based workload management directly within the ROS 2 executor. 
By reusing executor threads and introducing unified scheduling, ROSOMP optimizes resource utilization and ensures real-time performance. 
We developed the OMP Scheduler using the GCC 15.0.0 compiler with OpenMP 5.1 runtime support and implemented the ROSOMP Executor and OMP Orchestrator on the ROS 2 [Humble](https://docs.ros.org/en/humble/index.html) release.
# Getting Started
## Description
This repository contains two main parts:
<ul>
<li>libgomp : The libgomp module implements the OMP Scheduler based on the GCC compiler's OpenMP runtime support.</li>
<li>ROS 2 packages</li>
Our ROSOMP involves three ROS 2 packages: The `rcl` package and the `rclcpp` package for implementing the ROSOMP Executor and OMP Orchestrator, and `demo` code for testing the real-time performance of ROSOMP.
</ul>

## Prerequisites
ROSOMP is based on the latest version of ROS 2 Humble. Before compiling and building ROSOMP, you need to install ROS 2 on your Ubuntu system. Follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) for detailed instructions.

### Build

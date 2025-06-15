#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/syscall.h>
#include "stdio.h"
#include "string.h"

// D-NP     : ROS2_DEFAULT   + CLOSE
// D-GOMP   : ROS2_DEFAULT   + GOMP
// D-ROSOMP : ROS2_DEFAULT   + ROSOMP
// C-NP     : PICAS_MUTIL    + CLOSE
// C-GOMP   : PICAS_MUTIL    + GOMP
// C-ROSOMP : PICAS_MUTIL    + ROSOMP

// Using ROS 2 default multi-executor
#define ROS2_DEFAULT        0
// Using PICAS
#define PICAS_MUTIL         1
#define EXECUTE_TYPE        ROS2_DEFAULT

// Do not enable OpenMP
#define CLOSE               0
// Enable default OpenMP (BareOMP)
#define GOMP                1
// Enable ROSOMP
#define ROSOMP              2
#define OMP_TYPE            ROSOMP

// The number of executor threads
#define EXECUTOR_THREAD_NUM    4    //3,4,5,6
// The number of OpenMP threads
#define OPENMP_THREAD_NUM      4    //3,4,5,6

// The number of parallel regions in each OMP callback
#define OMP_NUM         1 // 1,2,3,4,5,6
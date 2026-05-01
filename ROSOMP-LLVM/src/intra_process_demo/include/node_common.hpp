#ifndef DEMOS__NODE_COMMON_H
#define DEMOS__NODE_COMMON_H

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/executors/conf.hpp"
#include "pthread.h"
#include "cmath"
#include "iostream"
#include <random>
// Replace with the correct path
#include "/home/neu/Disk/llvm-install/lib/clang/21/include/omp.h"

#define DUMMY_LOAD_ITER	      100

uint64_t dummy_load_calib = 1;

uint64_t get_clocktime() { 
    long int        ns; 
    uint64_t        all; 
    time_t          sec; 
    struct timespec spec; 

    clock_gettime(CLOCK_REALTIME, &spec);

    sec   = spec.tv_sec; 
    ns    = spec.tv_nsec; 
    all   = (uint64_t) sec * 1000000000UL + (uint64_t) ns; 
    return all;  
}

void dummy_load_ms(int load_ms) {
  volatile uint64_t i, j;
  for (j = 0; j < dummy_load_calib * load_ms; j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
}

void dummy_load_100us_new(int load_100us, bool use_openmp, rclcpp::executors::MultiThreadedExecutor &executor, uint32_t chain_idx, uint32_t node_idx, int priority, int omp_num) {
  (void) chain_idx;
  (void) node_idx;
  if(use_openmp){
      for(int i=1;i <= omp_num;++i){
        // Note that the number of external parameters within the parallel region must be consistent with `argv_num` in the `kmp_csupport.cpp` file.
        // For example, if there are currently two external parameters in the parallel region, `load_100us` and `omp_num`, then `argv_num` should be set to 2.
        // If `argv_num` does not match the number of external parameters in the parallel region under the ROSOMP configuration, it will cause the segmentation error!!!
        #pragma omp parallel num_threads(OPENMP_THREAD_NUM)
        {
          auto now = std::chrono::steady_clock::now();
          auto end_time = now + std::chrono::microseconds((int) ( (float)(load_100us) / OPENMP_THREAD_NUM / omp_num * 100 ) );
          while (std::chrono::steady_clock::now() < end_time)
          {
          }
        }
#if(OMP_TYPE == ROSOMP)
        // N-ROSOMP or C-ROSOMP
        int pri = omp_get_priority();
        // The lower the priority of a sub-callback in omp_queue, the more important it is.
        while(omp_queue_empty_check() != -1 && omp_queue_empty_check() <= pri){
          // ROSOMP Function
          if(omp_get_wait_for_work_flag() == true){
            // ROSOMP Function
            executor.trigger_interrupt_guard();
          }
          // ROSOMP Function
          omp_dequeue_and_execute();
        }
        // ROSOMP Function
        omp_synchronization(pri);
#endif
  }
  }else{
    auto now = std::chrono::steady_clock::now();
    auto end_time = now + std::chrono::microseconds((int) ( (float)(load_100us) / omp_num * 100 ));
    while (std::chrono::steady_clock::now() < end_time)
    {
    }
  }
}

int cnter_val[5] = {0, 0, 0, 0, 0};
struct Sensor : public rclcpp::Node {
  Sensor(const std::string & node_name, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, std::chrono::duration<int,std::milli> period_ms, bool use_openmp, 
  rclcpp::executors::MultiThreadedExecutor &executor, int priority, int omp_num)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    auto callback = [captured_pub, node_idx, chain_idx, exe_time_100us, use_openmp, &executor, priority, omp_num]() -> void {

        long int tid = syscall(SYS_gettid);
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
          return;
        }

        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = cnter_val[chain_idx-1]++;

        volatile uint64_t ts_start = get_clocktime();
        dummy_load_100us_new(exe_time_100us, use_openmp, executor, chain_idx, node_idx, priority, omp_num);
        volatile uint64_t ts_end = get_clocktime();
        int cpu_core = sched_getcpu();

        printf("|TID:%ld|-->[Chain%u-Sensor%u.exe:%u(100us).on:%d]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Published msg: %d]. \r\n\n", 
                tid,
                chain_idx,
                node_idx,
                exe_time_100us,
                cpu_core,
                ts_start,
                ts_end,
                ts_end-ts_start,
                msg->data
              );

        pub_ptr->publish(std::move(msg));

      };
    timer_ = this->create_wall_timer(period_ms, callback);
  }
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};


// MARK: Transfer Node structure definition.
struct Transfer : public rclcpp::Node {
  Transfer(const std::string & node_name, const std::string & input_topic, const std::string & output_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, bool use_openmp, 
  rclcpp::executors::MultiThreadedExecutor &executor, int priority, int omp_num)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output_topic, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [captured_pub, node_idx, chain_idx, exe_time_100us, use_openmp, &executor, priority, omp_num](std_msgs::msg::Int32::UniquePtr msg) {
          
          long int tid = syscall(SYS_gettid);
          auto pub_ptr = captured_pub.lock();
          if (!pub_ptr) {
            return;
          }
          volatile uint64_t ts_start = get_clocktime();
          dummy_load_100us_new(exe_time_100us, use_openmp, executor, chain_idx, node_idx, priority, omp_num);
          volatile uint64_t ts_end = get_clocktime();
          int cpu_core = sched_getcpu();

          printf("|TID:%ld|-->[Chain%u-Transfer%u.exe:%u(100us).on:%d]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Route msg: %d]. \r\n\n", 
                  tid,
                  chain_idx,
                  node_idx,
                  exe_time_100us,
                  cpu_core,
                  ts_start,
                  ts_end,
                  ts_end-ts_start,
                  msg->data
                );
                
          pub_ptr->publish(std::move(msg));
      }
    );
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

/// MARK: Command Node structure definition.
struct Command : public rclcpp::Node {
  Command(const std::string & node_name, const std::string & input_topic, uint32_t chain_idx, uint32_t node_idx, uint32_t exe_time_100us, bool use_openmp, 
  rclcpp::executors::MultiThreadedExecutor &executor, int priority, int omp_num)
  : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
    sub_ = this->create_subscription<std_msgs::msg::Int32>(
      input_topic,
      10,
      [node_idx, chain_idx, exe_time_100us, use_openmp, &executor, priority, omp_num](std_msgs::msg::Int32::UniquePtr msg) {

          long int tid = syscall(SYS_gettid);

          volatile uint64_t ts_start = get_clocktime();
          dummy_load_100us_new(exe_time_100us, use_openmp, executor, chain_idx, node_idx, priority, omp_num);
          volatile uint64_t ts_end = get_clocktime();
          int cpu_core = sched_getcpu();

          printf("|TID:%ld|-->[Chain%u-Command%u.exe:%u(100us).on:%d]-->[Start@%lu-End@%lu-Cost:%lu(ns) and Received msg: %d]. \r\n\n", 
                  tid,
                  chain_idx,
                  node_idx,
                  exe_time_100us,
                  cpu_core,
                  ts_start,
                  ts_end,
                  ts_end-ts_start,
                  msg->data
                );
          
      }
    );
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

#endif
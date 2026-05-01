// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <iostream>
#include <vector>
#include <cstdlib> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp/executors/conf.hpp"
#include "node_common.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), EXECUTOR_THREAD_NUM);

  auto Sub1 = std::make_shared<Command>("Sub1", "CHAIN1_TIMER_OUT", 1, 1, 0, true, executor, 99, OMP_NUM);
  executor.add_node(Sub1);
  auto Sub2 = std::make_shared<Command>("Sub2", "CHAIN1_TIMER_OUT", 1, 2, 0, true, executor, 98, OMP_NUM);
  executor.add_node(Sub2);
  auto Sub3 = std::make_shared<Command>("Sub3", "CHAIN1_TIMER_OUT", 1, 3, 0, true, executor, 97, OMP_NUM);
  executor.add_node(Sub3);
  auto Sub4 = std::make_shared<Command>("Sub4", "CHAIN1_TIMER_OUT", 1, 4, 0, true, executor, 96, OMP_NUM);
  executor.add_node(Sub4);
  auto Sub5 = std::make_shared<Command>("Sub5", "CHAIN1_TIMER_OUT", 1, 5, 0, true, executor, 95, OMP_NUM);
  executor.add_node(Sub5);
  auto Sub6 = std::make_shared<Command>("Sub6", "CHAIN1_TIMER_OUT", 1, 6, 0, true, executor, 94, OMP_NUM);
  executor.add_node(Sub6);
  auto Sub7 = std::make_shared<Command>("Sub7", "CHAIN1_TIMER_OUT", 1, 7, 0, true, executor, 93, OMP_NUM);
  executor.add_node(Sub7);
  auto Sub8 = std::make_shared<Command>("Sub8", "CHAIN1_TIMER_OUT", 1, 8, 0, true, executor, 92, OMP_NUM);
  executor.add_node(Sub8);
  auto Sub9 = std::make_shared<Command>("Sub9", "CHAIN1_TIMER_OUT", 1, 9, 0, true, executor, 91, OMP_NUM);
  executor.add_node(Sub9);
  auto Sub10 = std::make_shared<Command>("Sub10", "CHAIN1_TIMER_OUT", 1, 10, 0, true, executor, 90, OMP_NUM);
  executor.add_node(Sub10);

  auto Timer1 = std::make_shared<Sensor>("Timer1", "CHAIN1_TIMER_OUT", 1, 1, 0, 50ms, false, executor, 90, OMP_NUM);
  executor.add_node(Timer1);

#if(OMP_TYPE == CLOSE)
  omp_set_strategy(CLOSE);
#elif(OMP_TYPE == GOMP)
  omp_set_strategy(GOMP);
#elif(OMP_TYPE == ROSOMP)
  // ROSOMP Function
  omp_set_strategy(ROSOMP);
  // ROSOMP Function
  omp_set_executor_thread_num(EXECUTOR_THREAD_NUM);
  // ROSOMP Function
  omp_queue_init();
#endif

#if(EXECUTE_TYPE == PICAS_MUTIL)
  executor.set_callback_priority(Timer1->timer_, 90);
  executor.set_callback_priority(Sub1->sub_, 99);
  executor.set_callback_priority(Sub2->sub_, 98);
  executor.set_callback_priority(Sub3->sub_, 97);
  executor.set_callback_priority(Sub4->sub_, 96);
  executor.set_callback_priority(Sub5->sub_, 95);
  executor.set_callback_priority(Sub6->sub_, 94);
  executor.set_callback_priority(Sub7->sub_, 93);
  executor.set_callback_priority(Sub8->sub_, 92);
#endif

  executor.spin();

  rclcpp::shutdown();

  return 0;
}

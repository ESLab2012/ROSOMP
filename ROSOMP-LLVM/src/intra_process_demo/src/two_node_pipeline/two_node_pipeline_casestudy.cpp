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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "node_common.hpp"
#include "rclcpp/executors/conf.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), EXECUTOR_THREAD_NUM);

  auto Sub1 = std::make_shared<Command>("Sub1", "CHAIN1_TIMER_OUT", 1, 2, 201, true, executor, 99, 1);
  executor.add_node(Sub1);

  auto Transfer2_1 = std::make_shared<Transfer>("Transfer2_1", "CHAIN1_TIMER_OUT", "CHAIN2_Transfer_OUT", 2, 2, 291, true, executor, 99, 1);
  executor.add_node(Transfer2_1);

  auto Sub2 = std::make_shared<Command>("Sub2", "CHAIN2_Transfer_OUT", 2, 3, 42, false, executor, 99, 1);
  executor.add_node(Sub2);

  auto Transfer3_1 = std::make_shared<Transfer>("Transfer3_1", "CHAIN3_TIMER_OUT", "CHAIN3_Transfer_OUT", 3, 2, 210, true, executor, 99, 1);
  executor.add_node(Transfer3_1);

  auto Transfer3_2 = std::make_shared<Transfer>("Transfer3_2", "CHAIN3_Transfer_OUT", "CHAIN3_SUB_OUT", 3, 3, 24, false, executor, 99, 1);
  executor.add_node(Transfer3_2);

  auto Sub3 = std::make_shared<Command>("Sub3", "CHAIN3_SUB_OUT", 3, 4, 37, false, executor, 99, 1); 
  executor.add_node(Sub3);

  auto Transfer4 = std::make_shared<Transfer>("Transfer4_1", "CHAIN4_TIMER_OUT", "CHAIN4_SUB_OUT", 4, 2, 24, false, executor, 99, 1);
  executor.add_node(Transfer4);

  auto Sub4 = std::make_shared<Command>("Sub4", "CHAIN4_SUB_OUT", 4, 3, 31, false, executor, 99, 1);
  executor.add_node(Sub4);

  auto Transfer5 = std::make_shared<Transfer>("Transfer5", "CHAIN5_TIMER_OUT", "CHAIN5_SUB_OUT", 5, 2, 23, false, executor, 99, 1);
  executor.add_node(Transfer5);

  auto Sub5 = std::make_shared<Command>("Sub5", "CHAIN5_SUB_OUT", 5, 3, 26, false, executor, 99, 1);
  executor.add_node(Sub5);

  auto Sub6 = std::make_shared<Command>("Sub6", "CHAIN6_TIMER_OUT", 6, 2, 31, false, executor, 99, 1);
  executor.add_node(Sub6);

  auto Sub7 = std::make_shared<Command>("Sub7", "CHAIN7_TIMER_OUT", 7, 2, 37, false, executor, 99, 1);
  executor.add_node(Sub7);

  auto Timer1 = std::make_shared<Sensor>("Timer1", "CHAIN1_TIMER_OUT", 1, 1, 30, 50ms, false, executor, 99, 1);
  executor.add_node(Timer1);

  auto Timer3 = std::make_shared<Sensor>("Timer3", "CHAIN3_TIMER_OUT", 3, 1, 34, 100ms, false, executor, 99, 1);
  executor.add_node(Timer3);

  auto Timer4 = std::make_shared<Sensor>("Timer4", "CHAIN4_TIMER_OUT", 4, 1, 42, 100ms, false, executor, 99, 1);
  executor.add_node(Timer4);

  auto Timer5 = std::make_shared<Sensor>("Timer5", "CHAIN5_TIMER_OUT", 5, 1, 127, 100ms, false, executor, 99, 1);
  executor.add_node(Timer5);

  auto Timer6 = std::make_shared<Sensor>("Timer6", "CHAIN6_TIMER_OUT", 6, 1, 134, 150ms, false, executor, 99, 1);
  executor.add_node(Timer6);

  auto Timer7 = std::make_shared<Sensor>("Timer7", "CHAIN7_TIMER_OUT", 7, 1, 139, 200ms, false, executor, 99, 1);
  executor.add_node(Timer7);

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
  executor.set_callback_priority(Timer1->timer_, 91);
  executor.set_callback_priority(Sub1->sub_, 92);

  executor.set_callback_priority(Transfer2_1->sub_, 82);
  executor.set_callback_priority(Sub2->sub_, 83);

  executor.set_callback_priority(Timer3->timer_, 71);
  executor.set_callback_priority(Transfer3_1->sub_, 72);
  executor.set_callback_priority(Transfer3_2->sub_, 73);
  executor.set_callback_priority(Sub3->sub_, 74);

  executor.set_callback_priority(Timer4->timer_, 61);
  executor.set_callback_priority(Transfer4->sub_, 62);
  executor.set_callback_priority(Sub4->sub_, 63);

  executor.set_callback_priority(Timer5->timer_, 51);
  executor.set_callback_priority(Transfer5->sub_, 52);
  executor.set_callback_priority(Sub5->sub_, 53);

  executor.set_callback_priority(Timer6->timer_, 41);
  executor.set_callback_priority(Sub6->sub_, 42);

  executor.set_callback_priority(Timer7->timer_, 31);
  executor.set_callback_priority(Sub7->sub_, 32);
#endif

  executor.spin();

  rclcpp::shutdown();

  return 0;
}

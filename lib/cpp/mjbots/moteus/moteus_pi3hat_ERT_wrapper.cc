// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

// Modified by Loi Do, 2022, doloi@fel.cvut.cz


#include <sys/mman.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <future>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <memory>

#include "moteus_pi3hat_ERT_wrapper.h"

using namespace mjbots;
using MoteusInterface = moteus::Pi3HatMoteusInterface;


struct Arguments {
  // Default values
  int main_cpu = 1;
  int can_cpu = 2;
  double period_s = 0.01;
  int motor_id = 1;
  int motor_bus = 1;
};




/// This holds the user-defined control logic.
class SampleController {
 public:
  SampleController(const Arguments& arguments) : arguments_(arguments) {
  }

  /// This is called before any control begins, and must return the
  /// set of servos that are used, along with which bus each is
  /// attached to.
  std::vector<std::pair<int, int>> servo_bus_map() const {
    return {{ arguments_.motor_id, arguments_.motor_bus }};
  }

  /// This is also called before any control begins.  @p commands will
  /// be pre-populated with an entry for each servo returned by
  /// 'servo_bus_map'.  It can be used to perform one-time
  /// initialization like setting the resolution of commands and
  /// queries.
  void Initialize(std::vector<MoteusInterface::ServoCommand>* commands) {
    moteus::PositionResolution res;
    res.position = moteus::Resolution::kFloat;
    res.velocity = moteus::Resolution::kFloat;
    res.feedforward_torque = moteus::Resolution::kFloat;
    res.kp_scale = moteus::Resolution::kInt16;
    res.kd_scale = moteus::Resolution::kInt16;
    res.maximum_torque = moteus::Resolution::kIgnore;
    res.stop_position = moteus::Resolution::kIgnore;
    res.watchdog_timeout = moteus::Resolution::kIgnore;
    // For every motor, set the resolutions
    for (auto& cmd : *commands) {
      cmd.resolution = res;
    }

    for (const auto& id_bus_mappings : servo_bus_map()) {
      commands->push_back({});
      commands->back().id = id_bus_mappings.first;
      commands->back().bus = id_bus_mappings.second;
    }


  }

  moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply>& replies, int id, int bus) {
    for (const auto& item : replies) {
      if (item.id == id && item.bus == bus) { 
        return item.result; 
      }
    }
    return {};
  }

  /// These functions serve to control the motor.  @p status is the most recent
  /// status of all servos (note that it is possible for a given
  /// servo's result to be omitted on some frames).
  ///
  /// @p output should hold the desired output.  It will be
  /// pre-populated with the result of the last command cycle, (or
  /// Initialize to begin with).

  void SetPosition(std::vector<MoteusInterface::ServoCommand>* output, double position) {
    // const auto motor = Get(status, arguments_.motor_id, arguments_.motor_bus);
    // double motor_pos = motor.position;

    if (!std::isnan(motor_initial_pos_)) {     
      (*output)[0].mode = moteus::Mode::kPosition;
      (*output)[0].position.position = motor_initial_pos_ + position;  // Any starting position
    }
  }

  void ReadPosition(const std::vector<MoteusInterface::ServoReply>& status, double* out_position) {
    const auto motor = Get(status, arguments_.motor_id, arguments_.motor_bus);
    *out_position = motor.position;
  }

  void Stop(std::vector<MoteusInterface::ServoCommand>* output) {
    (*output)[0].mode = moteus::Mode::kStopped;   
  }

  void ResetInitPos(const std::vector<MoteusInterface::ServoReply>& status, std::vector<MoteusInterface::ServoCommand>* output) {
    (*output)[0].mode = moteus::Mode::kPosition;

    const auto motor = Get(status, arguments_.motor_id, arguments_.motor_bus);
    double motor_pos = motor.position;
    if (!std::isnan(motor_pos)) {
        motor_initial_pos_ = motor_pos;
    } 
  }

  // ----------------------------------------------
  // ------------------ Members: ------------------
  // ----------------------------------------------

  const Arguments arguments_;
  double motor_initial_pos_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<MoteusInterface::ServoCommand> commands; // Each entry in the vector is for each controlled motor.
  int num_motors = 1;

 private:
  
};


// ---------------------------------------------
// -------------- Global Variable --------------
// ---------------------------------------------

std::unique_ptr<SampleController> sample_controller;
std::unique_ptr<MoteusInterface> moteus_interface; 


class CAN_comm {
  public:
    CAN_comm() {
    }

    void read_and_write() {
      if (can_result.valid()) { // Just to avoid some runtime error.
        // Now we get the result of our last query and send off our new one.
        const auto current_values = can_result.get();

        // We copy out the results we just got out.
        const auto rx_count = current_values.query_result_size;
        saved_replies.resize(rx_count);
        std::copy(replies.begin(), replies.begin() + rx_count, saved_replies.begin());
      }

      // Then we can immediately ask them to be used again.
      auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
      moteus_interface->Cycle(
          moteus_data,
          [promise](const MoteusInterface::Output& output) {
            // This is called from an arbitrary thread, so we just set the promise value here.
            promise->set_value(output);
          });
      can_result = promise->get_future();
    }
  MoteusInterface::Data moteus_data; // This hold commands that we set using Controller and that is read by CAN interface
  std::future<MoteusInterface::Output> can_result; // To hold CAN communication result
  std::vector<MoteusInterface::ServoReply> replies{1}; // TODO: make the vector according to num of controlled motors
  std::vector<MoteusInterface::ServoReply> saved_replies;
};

CAN_comm can_comm_obj;


// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------

void setPosition(real64_T pos) {
  sample_controller->SetPosition(&sample_controller->commands, pos);
  can_comm_obj.read_and_write();
}

void readPosition(real64_T* out_pos) {
  double current_pos = 0;
  sample_controller->ReadPosition(can_comm_obj.saved_replies, &current_pos);
  std::cout << "Curr pos: " << current_pos << std::endl;
  *out_pos = current_pos;
}

void stopMotor() {
  for (int i=0; i < 5; i++) {
    // std::cout << "Stop motor" << std::endl;
    sample_controller->Stop(&sample_controller->commands);
    can_comm_obj.read_and_write();
  }
}

void Init_pi3hat(real64_T ctrl_period) {
  Arguments args;
  args.period_s = ctrl_period;
  //TODO: set other parameters

  // Initialize moteus interface. 
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = args.can_cpu;
  // This constructs the thread that sends and reads CAN messages
  moteus_interface = std::make_unique<MoteusInterface>(moteus_options);

  // Construct the controller
  sample_controller = std::make_unique<SampleController>(args);
  // Initialize (maybe, move to the contructor)
  sample_controller->Initialize(&sample_controller->commands); 

  // Connect the commands in the Controller with CAN communication
  can_comm_obj.moteus_data.commands = { sample_controller->commands.data(), sample_controller->commands.size() };
  can_comm_obj.moteus_data.replies = { can_comm_obj.replies.data(), can_comm_obj.replies.size() };

  // Set initial position, reset error states. We do it several times so it is really set (we might have to do it just 2 times)
  for (int i = 0; i < 5; i++) {
    sample_controller->ResetInitPos(can_comm_obj.saved_replies, &sample_controller->commands);
    can_comm_obj.read_and_write();
  }
}

// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------
// --------------------------------------------------------------------------------




void main_cycle() {
  // moteus::ConfigureRealtime(args.main_cpu); // Maybe not necessary when used with ERT linux
  
  const auto period = std::chrono::microseconds(static_cast<int64_t>(sample_controller->arguments_.period_s * 1e6));
  auto next_cycle = std::chrono::steady_clock::now() + period;

  const auto status_period = std::chrono::milliseconds(100);
  auto next_status = next_cycle + status_period;
  uint64_t cycle_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;

  // We will run at a fixed cycle time.
  uint64_t total_count = 100;
  while (cycle_count < total_count) {
    cycle_count++;
    margin_cycles++;

    const auto now = std::chrono::steady_clock::now();
    if (now > next_status) {
      const std::string modes = [&]() {
        std::ostringstream result;
        result.precision(4);
        result << std::fixed;
        for (const auto& item : can_comm_obj.saved_replies) {
          result << item.id << "/"
                  << item.bus << "/"
                  << static_cast<int>(item.result.mode) << "/"
                  << item.result.position << " ";
        }
        return result.str();
      }();

      next_status += status_period;
      total_margin = 0;
      margin_cycles = 0;
    }

    int skip_count = 0;
    while (now > next_cycle) {
      skip_count++;
      next_cycle += period;
    }

    // Wait for the next control cycle to come up.
    const auto pre_sleep = std::chrono::steady_clock::now();
    std::this_thread::sleep_until(next_cycle);
    const auto post_sleep = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
    total_margin += elapsed.count();
    next_cycle += period;

    setPosition(2);
  }
  stopMotor();
}



int main(int argc, char** argv) {
  

  // Lock memory for the whole process.
  // LockMemory();
  real64_T period_s = 0.01;
  Init_pi3hat(period_s);






  main_cycle();

  return 0;
}

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

/// @file
///
/// This is a simple application that demonstrates how to efficiently
/// monitor and control multiple moteus servos at a high rate using
/// the pi3hat.
///
/// It is contained in a single file for the purposes of
/// demonstration.  A real application should likely be implemented in
/// multiple translation units or structured for longer term
/// maintenance.

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

#include "moteus_protocol.h"
#include "pi3hat_moteus_interface.h"

using namespace mjbots;

using MoteusInterface = moteus::Pi3HatMoteusInterface;

namespace {

struct Arguments {
  Arguments(const std::vector<std::string>& args) {
    for (size_t i = 0; i < args.size(); i++) {
      const auto& arg = args[i];
      if (arg == "-h" || arg == "--help") {
        help = true;
      } else if (arg == "--main-cpu") {
        main_cpu = std::stoull(args.at(++i));
      } else if (arg == "--can-cpu") {
        can_cpu = std::stoull(args.at(++i));
      } else if (arg == "--period-s") {
        period_s = std::stod(args.at(++i));
      } else if (arg == "--motor-id") {
        motor_id = std::stoull(args.at(++i));
      } else if (arg == "--motor-bus") {
        motor_bus = std::stoull(args.at(++i));
      } else {
        throw std::runtime_error("Unknown argument: " + arg);
      }
    }
  }

  // Default values
  bool help = false;
  int main_cpu = 1;
  int can_cpu = 2;
  // TODO: change this so the "period_s" can be changed.
  double period_s = 0.01;
  int motor_id = 1;
  int motor_bus = 1;
};

// void DisplayUsage() {
//   std::cout << "Usage: moteus_control_example [options]\n";
//   std::cout << "\n";
//   std::cout << "  -h, --help           display this usage message\n";
//   std::cout << "  --main-cpu CPU       run main thread on a fixed CPU [default: 1]\n";
//   std::cout << "  --can-cpu CPU        run CAN thread on a fixed CPU [default: 2]\n";
//   std::cout << "  --period-s S         period to run control\n";
//   std::cout << "  --motor-id ID        servo ID of primary, undriven servo\n";
//   std::cout << "  --motor-bus BUS    bus of primary servo\n";
// }

// void LockMemory() {
//   // We lock all memory so that we don't end up having to page in
//   // something later which can take time.
//   {
//     const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
//     if (r < 0) {
//       throw std::runtime_error("Error locking memory");
//     }
//   }
// }

std::pair<double, double> MinMaxVoltage(const std::vector<MoteusInterface::ServoReply>& r) {
  double rmin = std::numeric_limits<double>::infinity();
  double rmax = -std::numeric_limits<double>::infinity();

  for (const auto& i : r) {
    if (i.result.voltage > rmax) { rmax = i.result.voltage; }
    if (i.result.voltage < rmin) { rmin = i.result.voltage; }
  }

  return std::make_pair(rmin, rmax);
}

/// This holds the user-defined control logic.
class SampleController {
 public:
  SampleController(const Arguments& arguments) : arguments_(arguments) {
  }

  // SampleController() {
  //   // Empty constructor. The members have default values.
  // }


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
  }

  moteus::QueryResult Get(const std::vector<MoteusInterface::ServoReply>& replies, int id, int bus) {
    for (const auto& item : replies) {
      if (item.id == id && item.bus == bus) { 
        return item.result; 
      }
    }
    return {};
  }

  /// This is run at each control cycle.  @p status is the most recent
  /// status of all servos (note that it is possible for a given
  /// servo's result to be omitted on some frames).
  ///
  /// @p output should hold the desired output.  It will be
  /// pre-populated with the result of the last command cycle, (or
  /// Initialize to begin with).
  void Run(const std::vector<MoteusInterface::ServoReply>& status, std::vector<MoteusInterface::ServoCommand>* output) {
    cycle_count_++;

    // This is where your control loop would go.

    if (cycle_count_ < 5) {
      for (auto& cmd : *output) {
        // We start everything with a stopped command to clear faults.
        cmd.mode = moteus::Mode::kStopped;
      }
    } else {
      // Then we control primary servo
      const auto motor = Get(status, arguments_.motor_id, arguments_.motor_bus);
      
      double motor_pos = motor.position;

      // Set primary position of the motor
      if (!std::isnan(motor_pos) && std::isnan(motor_initial_pos_)) {
        motor_initial_pos_ = motor_pos;
      } 

      if (!std::isnan(motor_initial_pos_)) {
        (*output)[0].mode = moteus::Mode::kStopped;        
        // (*output)[0].mode = moteus::Mode::kPosition;
        // (*output)[0].position.position = motor_initial_pos_ + double(cycle_count_) / 10000;
        // (*output)[0].position.velocity = 0.2;
      }
    }
  }

  void Stop(const std::vector<MoteusInterface::ServoReply>& status, std::vector<MoteusInterface::ServoCommand>* output) {
    cycle_count_++;
    (*output)[0].mode = moteus::Mode::kStopped;   
  }

  void ResetInitPos(const std::vector<MoteusInterface::ServoReply>& status, std::vector<MoteusInterface::ServoCommand>* output) {
    cycle_count_++;
    const auto motor = Get(status, arguments_.motor_id, arguments_.motor_bus);
    double motor_pos = motor.position;
    if (!std::isnan(motor_pos)) {
        motor_initial_pos_ = motor_pos;
    } 
  }

 private:
  const Arguments arguments_;
  uint64_t cycle_count_ = 0;
  double motor_initial_pos_ = std::numeric_limits<double>::quiet_NaN();
};


// ---------------------------------------------
// -------------- Global Variable --------------
// ---------------------------------------------

std::unique_ptr<SampleController> sample_controller;
std::unique_ptr<MoteusInterface> moteus_interface;

// -------------------- Runing the main cycle of the program --------------------

// template <typename Controller>
// void main_cycle(const Arguments& args, Controller* controller) {

void main_cycle(const Arguments& args) {
  if (args.help) {
    // DisplayUsage();
    return;
  }

  moteus::ConfigureRealtime(args.main_cpu); // Maybe not necessary when used with ERT linux


  // Initialize moteus_interface
  MoteusInterface::Options moteus_options;
  moteus_options.cpu = args.can_cpu;
  moteus_interface = std::make_unique<MoteusInterface>(moteus_options);

  // MoteusInterface moteus_interface{moteus_options};

  std::vector<MoteusInterface::ServoCommand> commands; // Each entry in the vector is for each controlled motor.
  for (const auto& id_bus_mappings : sample_controller->servo_bus_map()) {
    commands.push_back({});
    commands.back().id = id_bus_mappings.first;
    commands.back().bus = id_bus_mappings.second;
  }

  std::vector<MoteusInterface::ServoReply> replies{commands.size()};
  std::vector<MoteusInterface::ServoReply> saved_replies;

  sample_controller->Initialize(&commands);

  MoteusInterface::Data moteus_data;
  moteus_data.commands = { commands.data(), commands.size() };
  moteus_data.replies = { replies.data(), replies.size() };

  std::future<MoteusInterface::Output> can_result;

  const auto period = std::chrono::microseconds(static_cast<int64_t>(args.period_s * 1e6));
  auto next_cycle = std::chrono::steady_clock::now() + period;

  const auto status_period = std::chrono::milliseconds(100);
  auto next_status = next_cycle + status_period;
  uint64_t cycle_count = 0;
  double total_margin = 0.0;
  uint64_t margin_cycles = 0;

  // We will run at a fixed cycle time.
  while (true) {
    cycle_count++;
    margin_cycles++;
    {
      const auto now = std::chrono::steady_clock::now();
      if (now > next_status) {
        // NOTE: iomanip is not a recommended pattern.  We use it here
        // simply to not require any external dependencies, like 'fmt'.
        const auto volts = MinMaxVoltage(saved_replies);
        const std::string modes = [&]() {
          std::ostringstream result;
          result.precision(4);
          result << std::fixed;
          for (const auto& item : saved_replies) {
            result << item.id << "/"
                   << item.bus << "/"
                   << static_cast<int>(item.result.mode) << "/"
                   << item.result.position << " ";
          }
          return result.str();
        }();
        std::cout << std::setprecision(6) << std::fixed
                  << "Cycles " << cycle_count
                  << "  margin: " << (total_margin / margin_cycles)
                  << std::setprecision(1)
                  << "  volts: " << volts.first << "/" << volts.second
                  << "  modes: " << modes
                  << "   \r";
        std::cout.flush();
        next_status += status_period;
        total_margin = 0;
        margin_cycles = 0;
      }

      int skip_count = 0;
      while (now > next_cycle) {
        skip_count++;
        next_cycle += period;
      }
      if (skip_count) {
        std::cout << "\nSkipped " << skip_count << " cycles\n";
      }
    }
    // Wait for the next control cycle to come up.
    {
      const auto pre_sleep = std::chrono::steady_clock::now();
      std::this_thread::sleep_until(next_cycle);
      const auto post_sleep = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed = post_sleep - pre_sleep;
      total_margin += elapsed.count();
    }
    next_cycle += period;

    // ------------------------------------- HERE, we are calling the controller -------------------------------------
    sample_controller->Run(saved_replies, &commands);


    if (can_result.valid()) {
      // Now we get the result of our last query and send off our new
      // one.
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
          // This is called from an arbitrary thread, so we just set
          // the promise value here.
          promise->set_value(output);
        });
    can_result = promise->get_future();
  }
}
}






int main(int argc, char** argv) {
  Arguments args({argv + 1, argv + argc});

  // Lock memory for the whole process.
  // LockMemory();

  // SampleController sample_controller{args};
  sample_controller = std::make_unique<SampleController>(args);

  main_cycle(args);

  return 0;
}

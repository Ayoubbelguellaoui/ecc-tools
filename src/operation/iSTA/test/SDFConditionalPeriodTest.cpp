#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "DataManager.hpp"
#include "Logger.hpp"
#include "SDFWriter.hpp"

using namespace ista;

int main()
{
  const std::filesystem::path file = std::filesystem::temp_directory_path() / ("ista_period_" + std::to_string(getpid()) + ".sdf");
  Logger::initInst();
  DataManager::initInst();
  SDFWriter::initInst();
  try {
    Database& database = STADM.getDatabase();
    database.set_design_name("TOP");
    Instance& instance = database.get_instance_map()["memory"];
    instance.set_instance_name("memory");
    instance.set_cell_name("MEM");
    TimingCell& cell = database.get_timing_library().get_cell_map()["MEM"];
    for (const std::string port : {"CLK", "CLK_UNCOND"}) {
      Pin& pin = database.get_pin_map()["memory:" + port];
      pin.set_pin_name(port);
      pin.set_instance_name("memory");
      pin.set_full_name("memory:" + port);
      pin.set_direction(PinDirection::kInput);
    }
    TimingCheckArc period;
    period.set_check_type(TimingCheckType::kPeriod);
    period.set_data_port("CLK");
    // Distinct read/write requirements on the same pin must retain their enables.
    for (const auto& [condition, value] : {std::pair{"sdfcond_WCLK", 2.0}, std::pair{"sdfcond_RCLK", 2.5}}) {
      TimingArc timing_arc;
      timing_arc.set_sdf_cond(condition);
      timing_arc.get_check_table_map()[TransType::kRise].set_value_list({value});
      period.get_timing_arc_list().push_back(timing_arc);
    }
    cell.get_sdf_check_arc_list().push_back(period);
    period.set_data_port("CLK_UNCOND");
    period.get_timing_arc_list().clear();
    period.set_check_time(3.0);
    cell.get_sdf_check_arc_list().push_back(period);
    STASW.write(file.string());
    std::ifstream input(file);
    std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    for (const std::string expected : {"(PERIOD (COND sdfcond_WCLK CLK) (2.0000000000::2.0000000000))",
                                       "(PERIOD (COND sdfcond_RCLK CLK) (2.5000000000::2.5000000000))",
                                       "(PERIOD CLK_UNCOND (3.0000000000::3.0000000000))"}) {
      if (text.find(expected) == std::string::npos) {
        throw std::runtime_error("Missing SDF check: " + expected);
      }
    }
    if (text.find("(PERIOD CLK ") != std::string::npos) {
      throw std::runtime_error("Conditional period exported as unconditional");
    }
    std::filesystem::remove(file);
    std::cout << "SDF conditional and unconditional period checks: PASS\n";
  } catch (const std::exception& error) {
    std::filesystem::remove(file);
    std::cerr << error.what() << '\n';
    return 1;
  }
  SDFWriter::destroyInst();
  DataManager::destroyInst();
  Logger::destroyInst();
  return 0;
}

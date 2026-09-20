#include <unistd.h>

#include "DataManager.hpp"
#include "Logger.hpp"
#include "SDFWriter.hpp"

using namespace ista;

int main()
{
  const std::filesystem::path file = std::filesystem::temp_directory_path() / ("ista_sdf_slew_" + std::to_string(getpid()) + ".sdf");
  Logger::initInst();
  DataManager::initInst();
  SDFWriter::initInst();
  try {
    Database& database = STADM.getDatabase();
    database.set_design_name("TOP");
    Instance& instance = database.get_instance_map()["ff"];
    instance.set_instance_name("ff");
    instance.set_cell_name("DFF");
    instance.set_is_sequential(true);
    instance.set_clock_pin_name("ff:CK");
    for (const std::string port : {"CK", "D", "D_PHYSICAL", "D_ZERO"}) {
      Pin& pin = database.get_pin_map()["ff:" + port];
      pin.set_pin_name(port);
      pin.set_instance_name("ff");
      pin.set_full_name("ff:" + port);
      pin.set_direction(PinDirection::kInput);
      TimingPoint& point = database.get_timing_point_map()["ff:" + port];
      point.set_is_clock_point(true);
      for (AnalysisType mode : {AnalysisType::kMin, AnalysisType::kMax}) {
        point.get_clock_slew_map()[mode][TransType::kRise] = 0.1;
        point.get_physical_clock_slew_map()[mode][TransType::kRise] = mode == AnalysisType::kMin ? 0.5 : 0.8;
        if (port == "D") {
          point.get_data_slew_map()[mode][TransType::kRise] = mode == AnalysisType::kMin ? 0.6 : 0.9;
        } else if (port == "D_ZERO") {
          point.get_data_slew_map()[mode][TransType::kRise] = 0.0;
        }
      }
    }
    TimingArc timing_arc;
    TimingTable& table = timing_arc.get_check_table_map()[TransType::kRise];
    // Constraint = 2 * reference slew + data slew.
    table.set_axis_list({{0.0, 1.0}, {0.0, 1.0}});
    table.set_value_list({0.0, 1.0, 2.0, 3.0});
    TimingCell& cell = database.get_timing_library().get_cell_map()["DFF"];
    for (const std::string port : {"D", "D_PHYSICAL", "D_ZERO"}) {
      TimingCheckArc check;
      check.set_check_type(TimingCheckType::kSetup);
      check.set_clock_port("CK");
      check.set_data_port(port);
      check.get_timing_arc_list().push_back(timing_arc);
      cell.get_sdf_check_arc_list().push_back(check);
    }
    TimingCheckArc width;
    width.set_check_type(TimingCheckType::kWidth);
    width.set_data_port("CK");
    width.get_timing_arc_list().push_back(timing_arc);
    cell.get_sdf_check_arc_list().push_back(width);
    STASW.write(file.string());
    std::ifstream input(file);
    std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    for (const std::string expected : {
             "(SETUP (posedge D) (posedge CK) (0.8000000000::1.1000000000))",
             "(SETUP (posedge D_PHYSICAL) (posedge CK) (0.7000000000::1.0000000000))",
             "(SETUP (posedge D_ZERO) (posedge CK) (0.2000000000::0.2000000000))",
             "(WIDTH (posedge CK) (0.3000000000::0.3000000000))"}) {
      if (text.find(expected) == std::string::npos) {
        throw std::runtime_error("Wrong slew used for SDF: " + expected);
      }
    }
    std::filesystem::remove(file);
  } catch (const std::exception& error) {
    std::filesystem::remove(file);
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << "SDF data, physical fallback, explicit zero and ideal reference slew: PASS\n";
  SDFWriter::destroyInst();
  DataManager::destroyInst();
  Logger::destroyInst();
  return 0;
}

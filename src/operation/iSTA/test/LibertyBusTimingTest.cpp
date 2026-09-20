#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "Lib.hh"
#include "LibParserCpp.hh"

namespace {
void require(bool condition, const std::string& message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void checkArc(idb::LibCell* cell, const std::string& pin, idb::LibArc::TimingType type, double value)
{
  auto arc = cell->findLibertyArcSet("CLK", pin.c_str(), type);
  require(arc.has_value(), "Missing CLK -> " + pin + " timing arc");
  require((*arc)->get_arcs().size() == 1, "Duplicate timing arc to " + pin);
  require(std::abs((*arc)->front()->getDelayOrConstrainCheckNs(idb::TransType::kRise, 0.1, 0.2) - value) < 1e-10, "Incorrect timing table on " + pin);
}

void checkMacro(idb::LibCell* cell)
{
  require(cell != nullptr, "SRAM cell not found");
  size_t checks = 0;
  for (const auto& [bus, width] : {std::pair{"A", 10}, {"D", 64}, {"WEB", 64}, {"MAR", 4}}) {
    for (int index = 0; index < width; ++index) {
      std::string name = std::string(bus) + "[" + std::to_string(index) + "]";
      idb::LibPort* port = cell->get_cell_port_or_port_bus(name.c_str());
      require(port && name == port->get_port_name(), "Incorrect SRAM member lookup: " + name);
      require(port->isInput() && port->get_port_cap() > 0.0, "Missing SRAM input attributes: " + name);
      for (auto type : {idb::LibArc::TimingType::kSetupRising, idb::LibArc::TimingType::kHoldRising}) {
        auto arc = cell->findLibertyArcSet("CLK", name.c_str(), type);
        require(arc.has_value(), "Missing SRAM check: " + name);
        require((*arc)->front()->get_table_model() != nullptr, "Missing SRAM check table: " + name);
        ++checks;
      }
    }
  }
  for (int index = 0; index < 64; ++index) {
    std::string name = "Q[" + std::to_string(index) + "]";
    idb::LibPort* port = cell->get_cell_port_or_port_bus(name.c_str());
    require(port && name == port->get_port_name() && port->isOutput(), "Missing SRAM output: " + name);
    require(cell->findLibertyArcSet("CLK", name.c_str(), idb::LibArc::TimingType::kRisingEdge).has_value(), "Missing SRAM clock-to-Q arc: " + name);
  }
  std::cout << "SRAM bus coverage: " << checks << " setup/hold arc sets, 64 clock-to-Q outputs\n";
}
}  // namespace

int main(int argc, char** argv)
{
  const auto file = std::filesystem::temp_directory_path() / ("ista_bus_timing_" + std::to_string(getpid()) + ".lib");
  try {
    if (argc == 1) {
      std::ofstream out(file);
      out << R"(library(bus_test) {
        time_unit : "1ns";
        capacitive_load_unit(1,pf);
        cell(MEM) {
          pin(CLK) { direction : input; clock : true; }
          bus(A_2) {
            direction : input; capacitance : 0.06; max_transition : 0.4; fanout_load : 2.5;
            pin(A_2[5:3]) {
              timing() { related_pin : "CLK"; timing_type : setup_rising;
                rise_constraint(scalar) { values("0.25"); }
                fall_constraint(scalar) { values("0.3"); }
              }
              timing() { related_pin : "CLK"; timing_type : hold_rising;
                rise_constraint(scalar) { values("0.12"); }
                fall_constraint(scalar) { values("0.15"); }
              }
            }
          }
          bus(D) {
            direction : input; capacitance : 0.02;
            pin(D[9]) { capacitance : 0.09;
              timing() { related_pin : "CLK"; timing_type : setup_rising;
                rise_constraint(scalar) { values("0.9"); }
              }
            }
            pin(D[7]) {
              timing() { related_pin : "CLK"; timing_type : setup_rising;
                rise_constraint(scalar) { values("0.7"); }
              }
            }
          }
          bus(Q) {
            direction : output; max_capacitance : 0.8;
            pin(Q[2:4]) {
              timing() { related_pin : "CLK"; timing_type : rising_edge;
                cell_rise(scalar) { values("0.4"); }
                cell_fall(scalar) { values("0.5"); }
              }
            }
          }
          pin(CEB) { direction : input; capacitance : 0.03; }
        }
      })";
    }
    const std::string path = argc > 1 ? argv[1] : file.string();
    idb::LibertyReader reader(path.c_str());
    require(reader.readLib() && reader.linkLib(), "Liberty parse failed");
    std::unique_ptr<idb::LibBuilder> builder(reader.get_library_builder());
    if (argc > 1) {
      checkMacro(builder->get_lib()->findCell("ICS55_LVT_L0_R0_F1_X1024Y4D64_BW"));
      return 0;
    }
    idb::LibCell* cell = builder->get_lib()->findCell("MEM");
    for (int index = 3; index <= 5; ++index) {
      const std::string name = "A_2[" + std::to_string(index) + "]";
      idb::LibPort* port = cell->get_cell_port_or_port_bus(name.c_str());
      require(port && name == port->get_port_name(), "Wrong bus index lookup: " + name);
      require(port->isInput() && port->get_port_cap() == 0.06 && port->get_fanout_load() == 2.5, "Lost inherited bus attributes: " + name);
      require(port->get_port_slew_limit(idb::AnalysisMode::kMax) == 0.4, "Lost bus slew limit");
      checkArc(cell, name, idb::LibArc::TimingType::kSetupRising, 0.25);
      checkArc(cell, name, idb::LibArc::TimingType::kHoldRising, 0.12);
    }
    require(!cell->get_cell_port_or_port_bus("A_2[2]"), "Out-of-range bit aliased another port");
    require(cell->get_cell_port_or_port_bus("D[9]")->get_port_cap() == 0.09, "Bit override lost");
    require(cell->get_cell_port_or_port_bus("D[7]")->get_port_cap() == 0.02, "Bit override leaked into sibling");
    checkArc(cell, "D[9]", idb::LibArc::TimingType::kSetupRising, 0.9);
    checkArc(cell, "D[7]", idb::LibArc::TimingType::kSetupRising, 0.7);
    for (int index = 2; index <= 4; ++index) {
      const std::string name = "Q[" + std::to_string(index) + "]";
      idb::LibPort* port = cell->get_cell_port_or_port_bus(name.c_str());
      require(port && name == port->get_port_name() && port->isOutput(), "Ascending bus missing: " + name);
      require(port->get_port_cap_limit(idb::AnalysisMode::kMax) == 0.8, "Lost bus load limit");
      checkArc(cell, name, idb::LibArc::TimingType::kRisingEdge, 0.4);
    }
    require(cell->get_cell_port_or_port_bus("CEB")->get_port_cap() == 0.03, "Bus context leaked into scalar pin");
    std::filesystem::remove(file);
    std::cout << "Liberty bus timing: PASS\n";
  } catch (const std::exception& error) {
    std::filesystem::remove(file);
    std::cerr << error.what() << '\n';
    return 1;
  }
}

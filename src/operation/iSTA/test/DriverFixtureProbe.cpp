#include "DataManager.hpp"
#include "DelayCalculator.hpp"
#include "Logger.hpp"

using namespace ista;

int main(int argc, char** argv)
{
  if (argc != 4) {
    return 2;
  }
  Logger::initInst();
  DataManager::initInst();
  DelayCalculator::initInst();
  STADM.getConfig().min_slew_degradation = std::stoi(argv[3]);
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2]);
  std::string line;
  while (std::getline(input, line)) {
    auto f = nlohmann::json::parse(line);
    if (!f.contains("net") || !f.contains("loads")) {
      continue;
    }
    const std::string driver = f.at("output_pin"), net_name = f.at("net");
    if (driver.find("u_macro:") != std::string::npos && driver.find(":Q[61]") == std::string::npos) {
      continue;
    }
    STADC.init();
    Database& db = STADM.getDatabase();
    db = Database();
    AnalysisType mode = f.at("mode") == "min" ? AnalysisType::kMin : AnalysisType::kMax;
    TransType transition = f.at("output_transition") == "rise" ? TransType::kRise : TransType::kFall;
    TransType input_transition = f.at("input_transition") == "rise" ? TransType::kRise : TransType::kFall;
    TimingCellArc cell;
    TimingArc arc;
    arc.set_sense(transition == input_transition ? TimingArcSense::kPositive : TimingArcSense::kNegative);
    arc.set_library_name(f.at("library"));
    arc.set_time_unit_scale(f.at("time_scale"));
    arc.set_cap_unit_scale(f.at("cap_scale"));
    arc.set_slew_derate(f.at("derate"));
    auto thresholds = f.at("thresholds");
    arc.set_slew_lower_threshold_pct_rise(thresholds[0]);
    arc.set_slew_upper_threshold_pct_rise(thresholds[1]);
    arc.set_slew_lower_threshold_pct_fall(thresholds[2]);
    arc.set_slew_upper_threshold_pct_fall(thresholds[3]);
    arc.set_output_threshold_pct_rise(thresholds[4]);
    arc.set_output_threshold_pct_fall(thresholds[5]);
    for (const std::string key : {"delay", "slew"}) {
      TimingTable& table = key == "delay" ? arc.get_delay_table_map()[transition] : arc.get_slew_table_map()[transition];
      table.set_axis_list(f[key]["axes"].get<std::vector<std::vector<double>>>());
      table.set_value_list(f[key]["values"].get<std::vector<double>>());
      table.set_variable_type1(static_cast<TimingTableVariableType>(f[key]["var1"].get<int>()));
      table.set_variable_type2(static_cast<TimingTableVariableType>(f[key]["var2"].get<int>()));
    }
    cell.get_timing_arc_list().push_back(arc);
    Net& net = db.get_net_map()[net_name];
    net.set_net_name(net_name);
    net.set_driver_pin(driver);
    Pin& dp = db.get_pin_map()[driver];
    dp.set_full_name(driver);
    dp.set_pin_name("Y");
    dp.set_net_name(net_name);
    dp.set_direction(PinDirection::kOutput);
    ParasiticNet& rc = db.get_parasitic_library().get_net_map()[net_name];
    rc.set_net_name(net_name);
    for (auto& [name, cap] : f.at("nodes").items()) {
      auto& n = rc.get_node_map()[name];
      n.set_node_name(name);
      n.set_capacitance(cap);
    }
    for (auto& e : f.at("resistors")) {
      ParasiticResistor r;
      r.set_source_node(e[0]);
      r.set_sink_node(e[1]);
      r.set_resistance(e[2]);
      rc.get_resistor_list().push_back(r);
    }
    for (auto& [name, data] : f.at("loads").items()) {
      net.get_load_pin_list().push_back(name);
      auto& pin = db.get_pin_map()[name];
      pin.set_full_name(name);
      pin.set_pin_name("A");
      pin.set_instance_name(name);
      pin.set_net_name(net_name);
      pin.set_direction(PinDirection::kInput);
      db.get_instance_map()[name].set_cell_name(name);
      auto& load = db.get_timing_library().get_cell_map()[name];
      load.set_library_name(data.at("library"));
      load.get_port_map()["A"].set_capacitance(data.at("capacitance"));
      auto t = data.at("thresholds");
      load.set_input_threshold_pct_rise(t[0]);
      load.set_input_threshold_pct_fall(t[1]);
      load.set_slew_lower_threshold_pct_rise(t[2]);
      load.set_slew_upper_threshold_pct_rise(t[3]);
      load.set_slew_lower_threshold_pct_fall(t[4]);
      load.set_slew_upper_threshold_pct_fall(t[5]);
      load.set_slew_derate_from_library(t[6]);
    }
    DCTask task;
    task.set_proc_type(DCProcType::kCalculate);
    task.set_timing_cell_arc(&cell);
    task.set_output_pin(driver);
    task.set_analysis_type(mode);
    task.set_input_trans_type(input_transition);
    task.set_output_trans_type(transition);
    task.set_input_slew(f.at("input_slew"));
    STADC.calculate(task);
    f["recomputed_delay"] = task.get_timing_result().get_delay();
    f["recomputed_slew"] = task.get_timing_result().get_slew();
    for (auto& [name, data] : f.at("loads").items()) {
      Arc net_arc;
      net_arc.set_type(ArcType::kNet);
      net_arc.set_owner_name(net_name);
      net_arc.set_source_pin(driver);
      net_arc.set_sink_pin(name);
      DCTask wire;
      wire.set_proc_type(DCProcType::kCalculate);
      wire.set_arc(&net_arc);
      wire.set_analysis_type(mode);
      wire.set_input_trans_type(transition);
      wire.set_output_trans_type(transition);
      wire.set_input_slew(task.get_timing_result().get_slew());
      STADC.calculate(wire);
      f["recomputed_loads"][name] = {{"delay", wire.get_timing_result().get_delay()}, {"slew", wire.get_timing_result().get_slew()}};
    }
    output << f.dump() << '\n';
  }
  DelayCalculator::destroyInst();
  DataManager::destroyInst();
  Logger::destroyInst();
  return 0;
}

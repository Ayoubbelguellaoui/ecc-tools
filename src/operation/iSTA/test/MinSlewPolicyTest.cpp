#include "DataManager.hpp"
#include "DelayCalculator.hpp"
#include "Logger.hpp"

using namespace ista;

void require(bool condition, const std::string& label)
{
  if (!condition) {
    throw std::runtime_error(label);
  }
}

void near(double actual, double expected, const std::string& label)
{
  require(std::isfinite(actual) && std::abs(actual - expected) < 1E-9, label);
}

TimingCellArc makeCell()
{
  TimingCellArc cell;
  TimingArc arc;
  arc.set_library_name("driver_lib");
  arc.set_sense(TimingArcSense::kPositive);
  for (TransType transition : {TransType::kRise, TransType::kFall}) {
    double resistance = transition == TransType::kRise ? 0.8 : 0.4;
    TimingTable& delay = arc.get_delay_table_map()[transition];
    delay.set_axis_list({{0.0, 1.0}, {0.0, 1.0}});
    delay.set_value_list({0.02, 0.02 + resistance, 0.02, 0.02 + resistance});
    TimingTable& slew = arc.get_slew_table_map()[transition];
    slew.set_axis_list({{0.0, 1.0}, {0.0, 1.0}});
    slew.set_value_list({0.04, 0.04 + 1.5 * resistance, 0.04, 0.04 + 1.5 * resistance});
  }
  cell.get_timing_arc_list().push_back(arc);
  return cell;
}

void network(const std::string& topology)
{
  STADC.init();
  Database& db = STADM.getDatabase();
  db = Database();
  for (const std::string name : {"driver:Y", "load:A", "load_b:B"}) {
    Pin& pin = db.get_pin_map()[name];
    pin.set_full_name(name);
    pin.set_pin_name(name.substr(name.find(':') + 1));
    pin.set_instance_name(name.substr(0, name.find(':')));
    pin.set_net_name("n");
    pin.set_direction(name == "driver:Y" ? PinDirection::kOutput : PinDirection::kInput);
  }
  db.get_instance_map()["load"].set_cell_name("LOAD");
  db.get_instance_map()["load_b"].set_cell_name("COMPATIBLE_LOAD");
  db.get_timing_library().get_cell_map()["COMPATIBLE_LOAD"].set_library_name("driver_lib");
  db.get_timing_library().get_cell_map()["LOAD"].set_library_name("driver_lib");
  Net& net = db.get_net_map()["n"];
  net.set_net_name("n");
  net.set_driver_pin("driver:Y");
  net.set_load_pin_list({"load:A", "load_b:B"});
  ParasiticNet& rc = db.get_parasitic_library().get_net_map()["n"];
  rc.set_net_name("n");
  for (const std::string name : {"driver:Y", "load:A", "load_b:B"}) {
    rc.get_node_map()[name].set_node_name(name);
    rc.get_node_map()[name].set_capacitance(name == "driver:Y" ? 0.01 : 0.05);
  }
  ParasiticResistor edge;
  edge.set_source_node("driver:Y");
  edge.set_sink_node("load:A");
  edge.set_resistance(300.0);
  rc.get_resistor_list().push_back(edge);
  edge.set_source_node(topology == "series" ? "load:A" : "driver:Y");
  edge.set_sink_node("load_b:B");
  rc.get_resistor_list().push_back(edge);
  if (topology == "single") {
    rc.get_node_map()["load_b:B"].set_capacitance(0.0);
  } else if (topology == "loop") {
    edge.set_source_node("load:A");
    rc.get_resistor_list().push_back(edge);
  }
}

std::pair<DCTimingResult, DCTimingResult> calculate(TimingCellArc& cell, AnalysisType mode, TransType transition, const std::string& load_pin = "load:A")
{
  DCTask gate;
  gate.set_proc_type(DCProcType::kCalculate);
  gate.set_timing_cell_arc(&cell);
  gate.set_output_pin("driver:Y");
  gate.set_analysis_type(mode);
  gate.set_input_trans_type(transition);
  gate.set_output_trans_type(transition);
  gate.set_input_slew(0.02);
  STADC.calculate(gate);
  require(gate.get_is_valid(), "Invalid cell result");
  Arc arc;
  arc.set_type(ArcType::kNet);
  arc.set_owner_name("n");
  arc.set_source_pin("driver:Y");
  arc.set_sink_pin(load_pin);
  DCTask wire;
  wire.set_proc_type(DCProcType::kCalculate);
  wire.set_arc(&arc);
  wire.set_analysis_type(mode);
  wire.set_input_trans_type(transition);
  wire.set_output_trans_type(transition);
  wire.set_input_slew(gate.get_timing_result().get_slew());
  STADC.calculate(wire);
  require(wire.get_is_valid(), "Invalid wire result");
  return {gate.get_timing_result(), wire.get_timing_result()};
}

int main()
{
  Logger::initInst();
  DataManager::initInst();
  DelayCalculator::initInst();
  try {
    TimingCellArc cell = makeCell();
    require(STADM.getConfig().min_slew_degradation == 1, "Default must preserve waveform slew");
    network("single");
    auto [default_driver, default_load] = calculate(cell, AnalysisType::kMin, TransType::kFall);
    require(default_load.get_slew() > default_driver.get_slew() + 1E-5, "Default mode must retain degradation");
    STADM.getConfig().min_slew_degradation = 0;
    network("single");
    auto [rise_driver, rise_load] = calculate(cell, AnalysisType::kMin, TransType::kRise);
    auto [fall_driver, fall_load] = calculate(cell, AnalysisType::kMin, TransType::kFall);
    require(rise_load.get_slew() > rise_driver.get_slew() + 1E-5, "Rise must retain degradation");
    near(fall_load.get_slew(), fall_driver.get_slew(), "Fall must suppress degradation");
    auto [max_driver, max_load] = calculate(cell, AnalysisType::kMax, TransType::kFall);
    near(max_driver.get_slew(), fall_driver.get_slew(), "Driver waveform is independent of policy");
    near(max_load.get_delay(), fall_load.get_delay(), "Physical wire delay is independent of policy");
    require(max_load.get_slew() > max_driver.get_slew() + 1E-5, "Max analysis must retain degradation");

    // Both networks have 600 ohms summed resistance. Their admittance moments
    // give 150 and 405.6 ohms respectively: topology must change the decision.
    network("parallel");
    auto [parallel_driver, parallel_load] = calculate(cell, AnalysisType::kMin, TransType::kRise);
    require(parallel_load.get_slew() > parallel_driver.get_slew() + 1E-5, "Parallel branches are not a series resistance");
    network("series");
    auto [series_driver, series_load] = calculate(cell, AnalysisType::kMin, TransType::kRise);
    near(series_load.get_slew(), series_driver.get_slew(), "Series topology must suppress");

    STADM.getConfig().min_slew_degradation = 1;
    network("single");
    auto [reset_driver, reset_load] = calculate(cell, AnalysisType::kMin, TransType::kFall);
    near(reset_load.get_slew(), default_load.get_slew(), "Reinitialization clears the policy-dependent cache");
    STADM.getConfig().min_slew_degradation = 0;

    network("loop");
    auto [loop_driver, loop_load] = calculate(cell, AnalysisType::kMin, TransType::kFall);
    require(loop_load.get_slew() > loop_driver.get_slew() + 1E-5, "Tree reduction must not decide policy on a resistance loop");

    network("series");
    TimingCell& receiver = STADM.getDatabase().get_timing_library().get_cell_map()["LOAD"];
    receiver.set_library_name("receiver_lib");
    receiver.set_input_threshold_pct_fall(60.0);
    receiver.set_slew_lower_threshold_pct_fall(20.0);
    receiver.set_slew_upper_threshold_pct_fall(80.0);
    receiver.set_slew_derate_from_library(0.5);
    auto [mixed_driver, mixed_load] = calculate(cell, AnalysisType::kMin, TransType::kFall);
    auto [mixed_max_driver, mixed_max_load] = calculate(cell, AnalysisType::kMax, TransType::kFall);
    near(mixed_load.get_slew(), mixed_max_load.get_slew(), "Different conventions must retain the receiver waveform");
    require(mixed_load.get_slew() > 3.0 * mixed_driver.get_slew() + 1E-5, "Receiver degradation must survive conversion");
    auto [compatible_driver, compatible_load] = calculate(cell, AnalysisType::kMin, TransType::kFall, "load_b:B");
    near(compatible_load.get_slew(), compatible_driver.get_slew(), "Compatibility is checked per load");
    auto [compatible_max_driver, compatible_max_load] = calculate(cell, AnalysisType::kMax, TransType::kFall, "load_b:B");
    require(compatible_max_load.get_slew() > compatible_load.get_slew() + 1E-5, "Compatible receiver has physical RC degradation");
    near(mixed_load.get_delay(), mixed_max_load.get_delay(), "Threshold correction must use physical waveform");

    network("single");
    TimingCell& derated_receiver = STADM.getDatabase().get_timing_library().get_cell_map()["LOAD"];
    derated_receiver.set_library_name("derated_lib");
    derated_receiver.set_slew_derate_from_library(0.5);
    auto [derated_driver, derated_load] = calculate(cell, AnalysisType::kMin, TransType::kFall);
    auto [derated_max_driver, derated_max_load] = calculate(cell, AnalysisType::kMax, TransType::kFall);
    near(derated_load.get_slew(), derated_max_load.get_slew(), "Derate alone requires retaining the receiver waveform");

    network("single");
    TimingCell& shifted_receiver = STADM.getDatabase().get_timing_library().get_cell_map()["LOAD"];
    shifted_receiver.set_library_name("shifted_lib");
    shifted_receiver.set_slew_lower_threshold_pct_fall(20.0);
    shifted_receiver.set_slew_upper_threshold_pct_fall(60.0);
    auto [shifted_driver, shifted_load] = calculate(cell, AnalysisType::kMin, TransType::kFall);
    auto [shifted_max_driver, shifted_max_load] = calculate(cell, AnalysisType::kMax, TransType::kFall);
    near(shifted_load.get_slew(), shifted_max_load.get_slew(), "Equal threshold spans alone are not equivalent conventions");

    network("single");
    TimingCell& percent_receiver = STADM.getDatabase().get_timing_library().get_cell_map()["LOAD"];
    percent_receiver.set_library_name("percent_lib");
    percent_receiver.set_slew_lower_threshold_pct_fall(30.0);
    percent_receiver.set_slew_upper_threshold_pct_fall(70.0);
    auto [percent_driver, percent_load] = calculate(cell, AnalysisType::kMin, TransType::kFall);
    near(percent_load.get_slew(), percent_driver.get_slew(), "Equivalent thresholds across libraries allow the approximation");
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  DelayCalculator::destroyInst();
  DataManager::destroyInst();
  Logger::destroyInst();
  std::cout << "Minimum slew topology, transitions, max analysis, loops and receiver thresholds: PASS\n";
  return 0;
}

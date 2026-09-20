#include "DataManager.hpp"
#include "DelayCalculator.hpp"
#include "Logger.hpp"

using namespace ista;

void expectNear(double actual, double expected, const std::string& label)
{
  if (std::abs(actual - expected) > 1E-9 * std::max(1.0, std::abs(expected))) {
    throw std::runtime_error(label + ": actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
  }
}

void setNetwork(double resistance, double capacitance, bool ladder = false)
{
  STADC.init();
  Database& database = STADM.getDatabase();
  database.get_timing_library() = TimingLibrary();
  database.get_parasitic_library().get_net_map().clear();
  database.get_pin_map().clear();
  for (const std::string name : {"IN", "u:A"}) {
    Pin& pin = database.get_pin_map()[name];
    pin.set_full_name(name);
    pin.set_pin_name(name == "IN" ? "IN" : "A");
    pin.set_instance_name(name == "IN" ? "" : "u");
    pin.set_is_port(name == "IN");
    pin.set_direction(PinDirection::kInput);
  }
  Instance& instance = database.get_instance_map()["u"];
  instance.set_cell_name("LOAD");
  database.get_timing_library().get_cell_map()["LOAD"] = TimingCell();
  ParasiticNet& net = database.get_parasitic_library().get_net_map()["n"];
  net.set_net_name("n");
  net.get_node_map()["IN"].set_node_name("IN");
  net.get_node_map()["u:A"].set_node_name("u:A");
  net.get_node_map()["u:A"].set_capacitance(capacitance);
  ParasiticResistor edge;
  edge.set_source_node("IN");
  edge.set_sink_node("u:A");
  edge.set_resistance(resistance);
  net.get_resistor_list().push_back(edge);
  if (ladder) {
    net.get_node_map()["far"].set_node_name("far");
    net.get_node_map()["far"].set_capacitance(0.1);
    edge.set_source_node("u:A");
    edge.set_sink_node("far");
    edge.set_resistance(10000.0);
    net.get_resistor_list().push_back(edge);
  }
}

DCTimingResult calculate(double input_slew, TransType transition, AnalysisType analysis = AnalysisType::kMax)
{
  Arc arc;
  arc.set_type(ArcType::kNet);
  arc.set_owner_name("n");
  arc.set_source_pin("IN");
  arc.set_sink_pin("u:A");
  DCTask task;
  task.set_proc_type(DCProcType::kCalculate);
  task.set_arc(&arc);
  task.set_analysis_type(analysis);
  task.set_input_trans_type(transition);
  task.set_output_trans_type(transition);
  task.set_input_slew(input_slew);
  STADC.calculate(task);
  if (!task.get_is_valid()) {
    throw std::runtime_error("Invalid net calculation");
  }
  return task.get_timing_result();
}

int main()
{
  Logger::initInst();
  DataManager::initInst();
  DelayCalculator::initInst();
  try {
    for (double scale : {0.001, 1.0, 100.0}) {
      setNetwork(100.0 * scale, 0.01);
      double tau = 0.001 * scale;
      for (TransType transition : {TransType::kRise, TransType::kFall}) {
        for (AnalysisType analysis : {AnalysisType::kMin, AnalysisType::kMax}) {
          DCTimingResult result = calculate(0.0, transition, analysis);
          expectNear(result.get_delay(), tau * std::log(2.0), "Single RC step delay");
          expectNear(result.get_slew(), tau * std::log(0.7 / 0.3), "Single RC step slew");
        }
      }
    }
    setNetwork(100.0, 0.01, true);
    // Exact two-node nodal solution, evaluated independently: poles are
    // 0.9900892945709643 and 1010.0099107054291 / ns. Elmore is 0.011 ns.
    expectNear(calculate(0.0, TransType::kRise).get_delay(), 0.000696234355500187, "Near load on two-pole ladder");

    setNetwork(100.0, 0.01);
    STADM.getDatabase().get_parasitic_library().get_net_map()["n"].get_node_map()["IN"].set_capacitance(0.05);
    expectNear(calculate(0.0, TransType::kRise).get_delay(), 0.001 * std::log(2.0), "Ideal voltage source capacitance");

    setNetwork(1000.0, 0.05);
    STADM.getDatabase().get_timing_library().set_output_threshold_pct_rise(60.0);
    STADM.getDatabase().get_timing_library().set_output_threshold_pct_fall(60.0);
    TimingCell& cell = STADM.getDatabase().get_timing_library().get_cell_map()["LOAD"];
    cell.set_input_threshold_pct_rise(60.0);
    cell.set_input_threshold_pct_fall(40.0);
    cell.set_slew_lower_threshold_pct_rise(20.0);
    cell.set_slew_upper_threshold_pct_rise(80.0);
    cell.set_slew_lower_threshold_pct_fall(20.0);
    cell.set_slew_upper_threshold_pct_fall(80.0);
    cell.set_slew_derate_from_library(0.5);
    for (double slew : {0.001, 0.002, 0.001}) {
      // All receiver crossings occur after the source ramp ends. The exact
      // single-RC solution is tau * ln((tau/ramp)*expm1(ramp/tau)/remaining).
      double ramp = slew / 0.4;
      for (TransType transition : {TransType::kRise, TransType::kFall}) {
        double delay = 0.05 * std::log(0.05 / ramp * std::expm1(ramp / 0.05) / 0.4)
                       - ramp * (transition == TransType::kRise ? 0.6 : 0.4);
        DCTimingResult result = calculate(slew, transition);
        expectNear(result.get_delay(), delay, "Finite ramp with receiver threshold");
        expectNear(result.get_slew(), 0.05 * std::log(4.0) / 0.5, "Receiver slew thresholds and derate");
      }
    }
    setNetwork(1000.0, 0.05);
    // Crossings well inside a slow ramp approach the exact steady ramp lag R*C.
    for (double slew : {4.0, -4.0}) {
      DCTimingResult result = calculate(slew, TransType::kRise);
      expectNear(result.get_delay(), 0.05, "Slow ramp delay");
      expectNear(result.get_slew(), slew, "Slow ramp signed slew");
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  DelayCalculator::destroyInst();
  DataManager::destroyInst();
  Logger::destroyInst();
  std::cout << "Input port step, ladder, finite ramp, receiver thresholds and cache: PASS\n";
  return 0;
}

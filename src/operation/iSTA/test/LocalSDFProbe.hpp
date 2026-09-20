#pragma once

#include "DataManager.hpp"
#include "DelayCalculator.hpp"

namespace ista {

inline void localSDFProbe()
{
  const char* names_file = std::getenv("ISTA_LOCAL_PROBE_NAMES");
  const char* output_file = std::getenv("ISTA_LOCAL_PROBE_OUTPUT");
  if (names_file == nullptr || output_file == nullptr) {
    return;
  }
  std::ifstream input(names_file);
  std::set<std::string> names;
  std::string name;
  while (std::getline(input, name)) {
    names.insert(name);
  }
  std::ofstream output(output_file);
  std::ofstream fixtures(std::string(output_file) + ".jsonl");
  output << std::setprecision(17);
  output << "kind\towner\tfrom\tto\tmode\tin\tout\tarc_idx\tcondition\tdata_slew\tclock_slew\tphysical_slew\tload\tstored_delay\tfinal_slew\tmodel_delay\tmodel_slew\ttable_delay\ttable_slew\n";
  auto slewAt = [](auto& map, AnalysisType mode, TransType transition) {
    if (map.contains(mode) && map.at(mode).contains(transition)) {
      return map.at(mode).at(transition);
    }
    return std::numeric_limits<double>::quiet_NaN();
  };
  std::vector<std::unique_ptr<TimingCellArc>> retained_arcs;
  Database& database = STADM.getDatabase();
  for (Arc& arc : database.get_arc_list()) {
    Pin& source_pin = database.get_pin_map().at(arc.get_source_pin());
    if (!names.contains(arc.get_owner_name()) && !names.contains(source_pin.get_instance_name()) && !names.contains(arc.get_source_pin())) {
      continue;
    }
    TimingPoint& source = database.get_timing_point_map().at(arc.get_source_pin());
    TimingCellArc* cell_arc = arc.get_timing_cell_arc();
    if (cell_arc == nullptr) {
      continue;
    }
    for (AnalysisType mode : {AnalysisType::kMin, AnalysisType::kMax}) {
      for (auto& [arc_index, mode_map] : arc.get_timing_arc_delay_map()) {
        if (!mode_map.contains(mode)) {
          continue;
        }
        for (auto& [input_transition, output_map] : mode_map.at(mode)) {
          for (auto& [output_transition, stored_delay] : output_map) {
            retained_arcs.push_back(std::make_unique<TimingCellArc>(*cell_arc));
            TimingCellArc& selected = *retained_arcs.back();
            selected.get_timing_arc_list().clear();
            for (TimingArc& candidate : cell_arc->get_timing_arc_list()) {
              if (candidate.get_arc_idx() == arc_index) {
                selected.get_timing_arc_list().push_back(candidate);
              }
            }
            if (selected.get_timing_arc_list().size() != 1) {
              continue;
            }
            TimingArc& timing = selected.get_timing_arc_list().front();
            double data_slew = slewAt(source.get_data_slew_map(), mode, input_transition);
            double clock_slew = slewAt(source.get_clock_slew_map(), mode, input_transition);
            double physical_slew = slewAt(source.get_physical_clock_slew_map(), mode, input_transition);
            double final_slew = std::isfinite(data_slew) ? data_slew : std::isfinite(physical_slew) ? physical_slew : 0.0;
            double load = STADC.getPowerOutputLoad(arc.get_sink_pin(), mode, output_transition);
            DCTask task;
            task.set_proc_type(DCProcType::kCalculate);
            task.set_timing_cell_arc(&selected);
            task.set_output_pin(arc.get_sink_pin());
            task.set_analysis_type(mode);
            task.set_input_trans_type(input_transition);
            task.set_output_trans_type(output_transition);
            task.set_input_slew(final_slew);
            STADC.calculate(task);
            if (!timing.get_delay_table_map().contains(output_transition) || !timing.get_slew_table_map().contains(output_transition)) {
              continue;
            }
            nlohmann::json fixture;
            fixture["output_pin"] = arc.get_sink_pin();
            fixture["cell_name"] = database.get_instance_map().at(arc.get_owner_name()).get_cell_name();
            fixture["mode"] = mode == AnalysisType::kMin ? "min" : "max";
            fixture["input_transition"] = input_transition == TransType::kRise ? "rise" : "fall";
            fixture["output_transition"] = output_transition == TransType::kRise ? "rise" : "fall";
            fixture["input_slew"] = final_slew;
            fixture["condition"] = timing.get_sdf_cond();
            fixture["stored_delay"] = stored_delay;
            fixture["model_delay"] = task.get_timing_result().get_delay();
            fixture["model_slew"] = task.get_timing_result().get_slew();
            fixture["time_scale"] = timing.get_time_unit_scale();
            fixture["cap_scale"] = timing.get_cap_unit_scale();
            fixture["derate"] = timing.get_slew_derate();
            fixture["library"] = timing.get_library_name();
            fixture["thresholds"] = {timing.get_slew_lower_threshold_pct_rise(), timing.get_slew_upper_threshold_pct_rise(),
              timing.get_slew_lower_threshold_pct_fall(), timing.get_slew_upper_threshold_pct_fall(), timing.get_output_threshold_pct_rise(),
              timing.get_output_threshold_pct_fall()};
            for (const std::string key : {"delay", "slew"}) {
              TimingTable& table = key == "delay" ? timing.get_delay_table_map().at(output_transition) : timing.get_slew_table_map().at(output_transition);
              fixture[key] = {{"axes", table.get_axis_list()}, {"values", table.get_value_list()}, {"var1", int(table.get_variable_type1())}, {"var2", int(table.get_variable_type2())}};
            }
            const std::string& net_name = database.get_pin_map().at(arc.get_sink_pin()).get_net_name();
            if (database.get_parasitic_library().get_net_map().contains(net_name)) {
              ParasiticNet& net = database.get_parasitic_library().get_net_map().at(net_name);
              fixture["net"] = net_name;
              for (auto& [node_name, node] : net.get_node_map()) {
                fixture["nodes"][node_name] = node.get_capacitance();
              }
              for (ParasiticResistor& resistor : net.get_resistor_list()) {
                fixture["resistors"].push_back({resistor.get_source_node(), resistor.get_sink_node(), resistor.get_resistance()});
              }
              Net& logical_net = database.get_net_map().at(net_name);
              for (const std::string& load_name : logical_net.get_load_pin_list()) {
                Pin& pin = database.get_pin_map().at(load_name);
                if (pin.get_is_port() || !database.get_instance_map().contains(pin.get_instance_name())) { continue; }
                TimingCell& load_cell = database.get_timing_library().get_cell_map().at(database.get_instance_map().at(pin.get_instance_name()).get_cell_name());
                TimingCellPort& port = load_cell.get_port_map().at(pin.get_pin_name());
                double cap = port.get_capacitance();
                if (port.get_trans_capacitance_map().contains(mode) && port.get_trans_capacitance_map().at(mode).contains(output_transition)) {
                  cap = port.get_trans_capacitance_map().at(mode).at(output_transition);
                }
                fixture["loads"][load_name] = {{"capacitance",cap},{"library",load_cell.get_library_name()},
                  {"thresholds", {load_cell.get_input_threshold_pct_rise(),load_cell.get_input_threshold_pct_fall(),
                   load_cell.get_slew_lower_threshold_pct_rise(),load_cell.get_slew_upper_threshold_pct_rise(),
                   load_cell.get_slew_lower_threshold_pct_fall(),load_cell.get_slew_upper_threshold_pct_fall(),load_cell.get_slew_derate_from_library()}}};
              }
            }
            fixtures << fixture.dump() << '\n';
            double table_delay = timing.get_delay_table_map().at(output_transition).findValue(final_slew * timing.get_time_unit_scale(), load * timing.get_cap_unit_scale()) / timing.get_time_unit_scale();
            double table_slew = timing.get_slew_table_map().at(output_transition).findValue(final_slew * timing.get_time_unit_scale(), load * timing.get_cap_unit_scale()) / timing.get_time_unit_scale();
            output << "cell\t" << arc.get_owner_name() << '\t' << arc.get_source_pin() << '\t' << arc.get_sink_pin() << '\t'
                   << (mode == AnalysisType::kMin ? "min" : "max") << '\t' << (input_transition == TransType::kRise ? "rise" : "fall") << '\t'
                   << (output_transition == TransType::kRise ? "rise" : "fall") << '\t' << arc_index << '\t' << timing.get_sdf_cond() << '\t'
                   << data_slew << '\t' << clock_slew << '\t' << physical_slew << '\t' << load << '\t' << stored_delay << '\t'
                   << final_slew << '\t' << task.get_timing_result().get_delay() << '\t' << task.get_timing_result().get_slew() << '\t'
                   << table_delay << '\t' << table_slew << '\n';
          }
        }
      }
    }
  }
}

}  // namespace ista

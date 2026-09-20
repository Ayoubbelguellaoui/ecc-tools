#include "ClockPropagator.hpp"
#include "DataManager.hpp"
#include "DelayCalculator.hpp"
#include "Logger.hpp"
#include "SdcCommand.hpp"
#include "SdcCommands.hpp"
#include "TimingAnalyzer.hpp"
#include "TimingCaseAnalysis.hpp"
#include "TimingPropagator.hpp"
#include "TimingReporter.hpp"
#include "Utility.hpp"

using namespace ista;

namespace {
void require(bool condition, const std::string& message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}
void near(double actual, double expected, const std::string& message)
{
  require(std::fabs(actual - expected) < 1e-8, message + ": " + std::to_string(actual) + " != " + std::to_string(expected));
}
void command(const std::string& text, bool success = true)
{
  int result = SdcCommand::getInst().evalString(text);
  require((result == TCL_OK) == success, text + ": " + Tcl_GetStringResult(SdcCommand::getInst().getInterp()));
}
void pin(const std::string& name, bool port, PinDirection direction)
{
  Pin& value = STADM.getDatabase().get_pin_map()[name];
  value.set_full_name(name);
  value.set_is_port(port);
  value.set_direction(direction);
  const std::size_t separator = name.rfind(':');
  value.set_pin_name(separator == std::string::npos ? name : name.substr(separator + 1));
  if (!port && separator != std::string::npos) {
    value.set_instance_name(name.substr(0, separator));
  }
  STADM.getDatabase().get_timing_point_map()[name];
}
void fixture()
{
  STADM.getDatabase() = Database();
  STADC.init();
  STADM.getDatabase().set_design_name("TOP");
  for (const std::string name : {"clock", "other", "a", "b"}) {
    pin(name, true, PinDirection::kInput);
  }
  for (const std::string name : {"x", "y", "exported"}) {
    pin(name, true, PinDirection::kOutput);
  }
  pin("merge:Z", false, PinDirection::kOutput);
  for (const std::string name : {"sync.s_dat[0]_0:D", "sync.s_dat[0]_1:D", "sync.s_dat[1]_0:D"}) {
    pin(name, false, PinDirection::kInput);
  }
  command("create_clock -name c -period 10 [get_ports clock]");
  command("create_clock -name d -period 100 [get_ports other]");
}
void testKivenEnvironmentConstraints()
{
  STADM.getDatabase() = Database();
  STADC.init();
  STADM.getDatabase().set_design_name("TOP");
  pin("a", true, PinDirection::kInput);
  pin("x", true, PinDirection::kOutput);

  command("create_clock -name __VIRTUAL_CLK__ -period 20");
  command("set_clock_uncertainty 0.25 [get_clocks __VIRTUAL_CLK__]");
  command("set_input_delay 4 -clock [get_clocks __VIRTUAL_CLK__] -add_delay [get_ports a]");
  command("set_output_delay 3 -clock [get_clocks __VIRTUAL_CLK__] -add_delay [get_ports x]");
  command("set_load -pin_load 0.0335 [get_ports x]");

  TimingConstraint& constraints = STADM.getDatabase().get_timing_constraint();
  TimingClock& clock = constraints.get_clock_map().at("__VIRTUAL_CLK__");
  require(clock.get_source_list().empty(), "virtual clock unexpectedly has a source");
  near(clock.get_setup_uncertainty(), 0.25, "virtual clock setup uncertainty");
  near(clock.get_hold_uncertainty(), 0.25, "virtual clock hold uncertainty");
  TimingPortConstraint& input = constraints.get_port_constraint_map().at("a");
  TimingPortConstraint& output = constraints.get_port_constraint_map().at("x");
  require(input.get_clock_name() == "__VIRTUAL_CLK__", "input delay lost its virtual clock");
  near(input.get_input_delay_max(), 4.0, "Kiven input delay");
  near(input.get_input_delay_min(), 4.0, "Kiven input delay");
  require(output.get_clock_name() == "__VIRTUAL_CLK__", "output delay lost its virtual clock");
  near(output.get_output_delay_max(), 3.0, "Kiven output delay");
  near(output.get_output_delay_min(), 3.0, "Kiven output delay");
  near(output.get_load(), 0.0335, "Kiven pin load");
  ClockPropagator::getInst().propagate();
}
void arc(const std::string& from, const std::string& to, double delay = 1.0)
{
  Database& db = STADM.getDatabase();
  Arc value;
  value.set_source_pin(from);
  value.set_sink_pin(to);
  value.set_delay(delay);
  value.set_delay_min(delay);
  value.set_delay_max(delay);
  for (AnalysisType mode : {AnalysisType::kMin, AnalysisType::kMax}) {
    for (TransType transition : {TransType::kRise, TransType::kFall}) {
      value.get_input_output_delay_map()[mode][transition][transition] = delay;
    }
  }
  db.get_outgoing_arc_list_map()[from].push_back(db.get_arc_list().size());
  db.get_incoming_arc_list_map()[to].push_back(db.get_arc_list().size());
  db.get_arc_list().push_back(value);
}
void dataGraph()
{
  Database& db = STADM.getDatabase();
  db.get_start_point_list() = {"a", "b"};
  db.get_end_point_list() = {"x", "y"};
  db.get_timing_order_list() = {"a", "b", "merge:Z", "x", "y"};
  arc("a", "merge:Z");
  arc("b", "merge:Z");
  arc("merge:Z", "x");
  arc("merge:Z", "y");
  command("set_input_delay -max 13 -clock c [get_ports a]");
  command("set_input_delay -min -3 -clock c [get_ports a]");
  command("set_input_delay -max 3 -clock c [get_ports b]");
  command("set_input_delay -min 1 -clock c [get_ports b]");
  command("set_output_delay 0 -clock c [get_ports {x y}]");
}
void testCaseAnalysisNoopWithoutConstraints()
{
  fixture();
  Database& db = STADM.getDatabase();
  pin("const0:A", false, PinDirection::kInput);
  pin("const0:Z", false, PinDirection::kOutput);

  Instance& instance = db.get_instance_map()["const0"];
  instance.set_instance_name("const0");
  instance.set_cell_name("CONST0_X1");
  instance.set_pin_name_list({"const0:A", "const0:Z"});

  TimingCell& cell = db.get_timing_library().get_cell_map()["CONST0_X1"];
  TimingCellPort& output = cell.get_port_map()["Z"];
  output.set_port_name("Z");
  output.set_is_output(true);
  LogicExpressionTerm zero;
  zero.set_operation_type(LogicOperationType::kZero);
  LogicExpression function;
  function.get_term_list().push_back(zero);
  output.set_function_expression(function);

  Arc cell_arc;
  cell_arc.set_source_pin("const0:A");
  cell_arc.set_sink_pin("const0:Z");
  cell_arc.set_owner_name("const0");
  cell_arc.set_library_source_port("A");
  cell_arc.set_library_sink_port("Z");
  cell_arc.set_type(ArcType::kCell);
  db.get_arc_list().push_back(cell_arc);

  TimingCaseAnalysis::apply(db);
  require(db.get_timing_constraint().get_effective_case_analysis_map().empty(), "empty case analysis inferred constants");
  require(!db.get_arc_list().back().get_is_case_analysis_disable(), "empty case analysis disabled a library-constant arc");

  db.get_arc_list().back().set_is_case_analysis_disable(true);
  TimingCaseAnalysis::apply(db);
  require(!db.get_arc_list().back().get_is_case_analysis_disable(), "empty case analysis left a stale disabled arc");
}
std::vector<TimingPath> analyze()
{
  Database& db = STADM.getDatabase();
  for (auto& [name, point] : db.get_timing_point_map()) {
    point.get_path_state_map().clear();
  }
  db.get_timing_path_group_list().clear();
  STATP.propagate();
  STATA.analyze();
  std::vector<TimingPath> result;
  for (TimingPathGroup& group : db.get_timing_path_group_list()) {
    for (auto& [name, endpoint] : group.get_timing_path_end_map()) {
      for (TimingPath& path : endpoint.get_timing_path_list()) {
        result.push_back(path);
      }
    }
  }
  return result;
}
TimingPath path(std::vector<TimingPath> paths, const std::string& end, AnalysisType mode)
{
  for (TimingPath& candidate : paths) {
    if (candidate.get_end_point() == end && candidate.get_analysis_type() == mode) {
      return candidate;
    }
  }
  throw std::runtime_error("missing path to " + end);
}
void testQueriesAndGeneratedClocks()
{
  fixture();
  command(R"(if {[llength [get_pins -regexp {.*sync\.s_dat\[0\]_[0-1]:D}]] != 2} {error raw_regex})");
  command(R"(if {[llength [get_pins -regexp [list {.*sync\.s_dat\[0\]_[0-1]/D}]]] != 2} {error list_regex})");
  command("if {[llength [get_pins {sync.s_dat[0]_0/D}]] != 1} {error literal_bus}");
  command("if {[llength [all_inputs -no_clocks]] != 2} {error inputs}");
  command("if {[llength [all_outputs]] != 3} {error outputs}");
  command("get_clocks missing", false);
  command("get_pins -quiet missing");
  command("create_generated_clock -name div -source [get_ports clock] -master_clock c -divide_by 4 [get_ports exported]");
  auto& clocks = STADM.getDatabase().get_timing_constraint().get_clock_map();
  near(clocks.at("div").get_period(), 40, "divide period");
  near(clocks.at("div").get_fall_edge(), 20, "divide fall");
  command("create_generated_clock -name inv -source clock -master_clock c -invert -add exported");
  near(clocks.at("inv").get_rise_edge(), 5, "inverted rise");
  command("create_generated_clock -name shifted -source clock -master_clock c -edges {1 3 5} -edge_shift {2 2 2} -add exported");
  near(clocks.at("shifted").get_period(), 20, "edge period");
  near(clocks.at("shifted").get_rise_edge(), 2, "edge shift");
  command("create_generated_clock -name twice -source clock -master_clock c -multiply_by 2 -duty_cycle 40 -add exported");
  near(clocks.at("twice").get_fall_edge(), 2, "multiply duty");
  const std::size_t size = clocks.size();
  command("create_generated_clock -name bad -source clock -master_clock c -divide_by 0 exported", false);
  command("create_generated_clock -name c -source exported -master_clock div exported", false);
  require(clocks.size() == size, "invalid generated clock mutated database");
  ClockPropagator::getInst().propagate();  // Several modes on a terminal clock output are legal.
}
void testExceptionsPreserveAlternatives()
{
  fixture();
  dataGraph();
  near(path(analyze(), "x", AnalysisType::kMax).get_slack(), -5, "baseline setup");
  near(path(analyze(), "x", AnalysisType::kMin).get_slack(), -1, "baseline hold");
  command("set_false_path -from [get_ports a] -to [get_ports x]");
  std::vector<TimingPath> paths = analyze();
  near(path(paths, "x", AnalysisType::kMax).get_slack(), 5, "valid alternative setup");
  near(path(paths, "x", AnalysisType::kMin).get_slack(), 3, "valid alternative hold");
  require(path(paths, "x", AnalysisType::kMax).get_start_point() == "b", "false worst start survived");
  near(path(paths, "y", AnalysisType::kMax).get_slack(), -5, "exception leaked to another endpoint");
  STADM.getDatabase().get_timing_constraint().get_false_path_list().clear();
  command("set_false_path -setup -from a -to x");
  paths = analyze();
  near(path(paths, "x", AnalysisType::kMax).get_slack(), 5, "setup-only alternative");
  near(path(paths, "x", AnalysisType::kMin).get_slack(), -1, "setup-only cut hold");
}
void testOrderedFalsePathExceptions()
{
  fixture();
  Database& db = STADM.getDatabase();
  for (const std::string name : {"slow_a:Z", "slow_b:Z", "bypass:Z"}) {
    pin(name, false, PinDirection::kOutput);
    const std::string instance_name = name.substr(0, name.find(':'));
    Instance& instance = db.get_instance_map()[instance_name];
    instance.set_instance_name(instance_name);
    instance.set_pin_name_list({name});
  }
  db.get_pin_map().at("slow_b:Z").set_net_name("slow_b_net");
  db.get_net_map()["slow_b_net"].set_net_name("slow_b_net");
  db.get_start_point_list() = {"a"};
  db.get_end_point_list() = {"x"};
  db.get_timing_order_list() = {"a", "slow_a:Z", "slow_b:Z", "bypass:Z", "x"};
  arc("a", "slow_a:Z", 4);
  arc("slow_a:Z", "slow_b:Z", 4);
  arc("slow_b:Z", "x", 4);
  arc("a", "bypass:Z", 1);
  arc("bypass:Z", "x", 1);
  command("set_input_delay 3 -clock c [get_ports a]");
  command("set_output_delay 0 -clock c [get_ports x]");

  TimingPath baseline = path(analyze(), "x", AnalysisType::kMax);
  require(baseline.get_path_delay() == 15, "baseline did not use slow path");
  command("set_false_path -from [get_ports a] -through [get_cells slow_b] -through [get_cells slow_a] -to [get_ports x]");
  require(path(analyze(), "x", AnalysisType::kMax).get_path_delay() == 15, "reversed through order cut the path");

  db.get_timing_constraint().get_false_path_list().clear();
  command(R"(set_false_path -from [get_ports a] -through [get_cells slow_a] -through [get_nets slow_b_net] \
                     -to [get_ports x] -comment {ordered through})");
  TimingPath bypass = path(analyze(), "x", AnalysisType::kMax);
  require(bypass.get_path_delay() == 5, "ordered through did not preserve the bypass path");
  require(bypass.get_point_list()[1].get_pin_name() == "bypass:Z", "false through path remained reportable");
  const TimingException& exception = db.get_timing_constraint().get_false_path_list().front();
  require(exception.get_through_list().size() == 2, "multiple through selectors lost their order");
  require(exception.get_comment() == "ordered through", "false path comment was not stored");

  db.get_timing_constraint().get_false_path_list().clear();
  command("set_false_path -rise_from [get_ports a] -rise_to [get_ports x]");
  require(path(analyze(), "x", AnalysisType::kMax).get_trans_type() == TransType::kFall, "rise endpoint false path cut the wrong transition");
  db.get_timing_constraint().get_false_path_list().clear();
  command("set_false_path -fall -from [get_ports a] -to [get_ports x]");
  require(path(analyze(), "x", AnalysisType::kMax).get_trans_type() == TransType::kRise, "-fall false path cut the rising transition");

  db.get_timing_constraint().get_false_path_list().clear();
  command("set_false_path -rise_through [get_pins slow_a/Z] -to [get_ports x]");
  TimingPath fall_through_survivor = path(analyze(), "x", AnalysisType::kMax);
  require(fall_through_survivor.get_trans_type() == TransType::kFall && fall_through_survivor.get_path_delay() == 15,
          "-rise_through cut the wrong path transition");
  db.get_timing_constraint().get_false_path_list().clear();
  command("set_false_path -fall_from [get_clocks c] -to [get_ports x]");
  require(path(analyze(), "x", AnalysisType::kMax).get_path_delay() == 15, "falling launch edge exception matched a rising clock");
  db.get_timing_constraint().get_false_path_list().clear();
  command("set_false_path -rise_to [get_clocks c]");
  require(analyze().empty(), "capture-clock edge exception left reportable paths");
  db.get_timing_constraint().get_false_path_list().clear();
  command("set_false_path -from a -rise_from a -to x", false);
  require(db.get_timing_constraint().get_false_path_list().empty(), "invalid false path mutated constraints");
}
void testPathDelayAndMulticycleExceptions()
{
  fixture();
  dataGraph();
  command("set_max_delay 4 -from [get_ports a] -to [get_ports x]");
  command("set_multicycle_path 5 -setup -from [get_ports a] -to [get_ports x]");
  TimingPath setup = path(analyze(), "x", AnalysisType::kMax);
  require(setup.get_start_point() == "a", "max delay did not keep the violating startpoint");
  near(setup.get_required_time(), 4, "max delay required time");
  near(setup.get_required_time_adjustment(), -6, "reported max delay adjustment");
  near(setup.get_slack(), -11, "max delay must override multicycle");
  const std::filesystem::path report_directory = std::filesystem::temp_directory_path() / "ista_sdc_path_exception_test";
  std::filesystem::create_directories(report_directory);
  const auto saved_report_config = STADM.getConfig();
  STADM.getConfig().tr_temp_directory_path = report_directory.string() + "/";
  STADM.getConfig().output_timing_reports = 1;
  STADM.getConfig().has_timing_report_slack_lesser_than = true;
  STADM.getConfig().timing_report_slack_lesser_than = 1000;
  TimingReporter::initInst();
  STATR.report();
  std::ifstream report(report_directory / "timing_max_in2out.rpt");
  const std::string report_text((std::istreambuf_iterator<char>(report)), std::istreambuf_iterator<char>());
  require(report_text.find("path exception adjustment") != std::string::npos, "timing report omitted path exception adjustment");
  STADM.getConfig() = saved_report_config;

  command("set_min_delay 2 -from [get_ports a] -to [get_ports x]");
  TimingPath hold = path(analyze(), "x", AnalysisType::kMin);
  require(hold.get_start_point() == "a", "min delay did not keep the violating startpoint");
  near(hold.get_required_time(), 2, "min delay required time");
  near(hold.get_slack(), -3, "min delay slack");
  command("set_false_path -setup -from [get_ports a] -to [get_ports x]");
  require(path(analyze(), "x", AnalysisType::kMax).get_start_point() == "b", "false path did not override path delay");

  fixture();
  dataGraph();
  command("set_multicycle_path 2 -setup -to [get_ports x]");
  setup = path(analyze(), "x", AnalysisType::kMax);
  near(setup.get_required_time(), 20, "setup multicycle required time");
  hold = path(analyze(), "x", AnalysisType::kMin);
  near(hold.get_required_time(), 10, "setup multicycle must move the default hold relation");
  command("set_multicycle_path 1 -hold -to [get_ports x]");
  hold = path(analyze(), "x", AnalysisType::kMin);
  near(hold.get_required_time(), 0, "hold multicycle compensation");
  near(hold.get_slack(), -1, "hold multicycle slack");

  fixture();
  dataGraph();
  command("set_output_delay 0 -clock d x");
  command("set_multicycle_path 2 -setup -to x");
  near(path(analyze(), "x", AnalysisType::kMax).get_required_time(), 110, "default multicycle must use endpoint period");
  STADM.getDatabase().get_timing_constraint().get_path_exception_list().clear();
  command("set_multicycle_path 2 -setup -start -to x");
  near(path(analyze(), "x", AnalysisType::kMax).get_required_time(), 20, "-start multicycle must use launch period");

  fixture();
  dataGraph();
  command("set_max_delay 3 -to x");
  command("set_max_delay 8 -from a -to x");
  command("set_max_delay 6 -from a -to x");
  setup = path(analyze(), "x", AnalysisType::kMax);
  near(setup.get_required_time(), 6, "specific and tighter max delay precedence");
  command("set_min_delay 2 -from a -to x");
  command("set_min_delay 4 -from a -to x");
  near(path(analyze(), "x", AnalysisType::kMin).get_required_time(), 4, "tighter min delay precedence");
  command("reset_path -setup -from a");
  near(path(analyze(), "x", AnalysisType::kMax).get_required_time(), 3, "reset_path must retain exceptions without a matching from selector");
  near(path(analyze(), "x", AnalysisType::kMin).get_required_time(), 4, "reset_path changed hold constraints");
  command("unset_path_exceptions -hold -to x");
  near(path(analyze(), "x", AnalysisType::kMin).get_required_time(), 0, "unset_path_exceptions hold dimension");

  fixture();
  dataGraph();
  command("set_false_path -from a -to x");
  command("reset_path -setup -from a");
  require(path(analyze(), "x", AnalysisType::kMax).get_start_point() == "a", "partial reset did not restore setup path");
  require(path(analyze(), "x", AnalysisType::kMin).get_start_point() == "b", "partial reset removed hold false path");
  command("unset_path_exceptions -hold -from a");
  require(path(analyze(), "x", AnalysisType::kMin).get_start_point() == "a", "hold reset did not restore path");

  fixture();
  dataGraph();
  command("set_false_path -from {a b} -to x");
  command("reset_path -setup -from a");
  const auto& partial_reset = STADM.getDatabase().get_timing_constraint().get_path_exception_list();
  require(std::any_of(partial_reset.begin(), partial_reset.end(), [](const TimingException& exception) {
            return exception.get_type() == TimingExceptionType::kFalsePath && exception.get_setup() && exception.get_from_objects().contains("b");
          }),
          "reset_path removed nonintersecting objects from a collection");

  fixture();
  dataGraph();
  command("set_false_path -from a -to x");
  command("reset_path -setup -rise -from a");
  const auto& transition_reset = STADM.getDatabase().get_timing_constraint().get_path_exception_list();
  require(std::any_of(transition_reset.begin(), transition_reset.end(), [](const TimingException& exception) {
            return exception.get_type() == TimingExceptionType::kFalsePath && !exception.get_setup() && exception.get_hold()
                   && exception.get_rise() && !exception.get_fall();
          }),
          "rise reset removed the hold dimension");
  require(std::any_of(transition_reset.begin(), transition_reset.end(), [](const TimingException& exception) {
            return exception.get_type() == TimingExceptionType::kFalsePath && exception.get_setup() && exception.get_hold()
                   && !exception.get_rise() && exception.get_fall();
          }),
          "rise reset removed the fall dimension");

  fixture();
  dataGraph();
  command("set_multicycle_path abc -to x", false);
  command("set_max_delay -to x", false);
  command("set_max_delay 4 -probe -from [get_pins merge/Z] -to x", false);
  require(STADM.getDatabase().get_timing_constraint().get_path_exception_list().empty(), "invalid path exception mutated constraints");
}
void testClockGroupsAndWaveforms()
{
  fixture();
  dataGraph();
  command("set_input_delay -max 3 -clock d b");
  command("set_input_delay -min 1 -clock d b");
  command("set_output_delay 0 -clock d x");
  command("set_clock_groups -asynchronous -group [get_clocks c] -group [get_clocks d]");
  std::vector<TimingPath> paths = analyze();
  require(path(paths, "x", AnalysisType::kMax).get_start_point() == "b", "cross-domain setup survived");
  require(path(paths, "y", AnalysisType::kMin).get_start_point() == "a", "same-domain hold was cut");
  auto& constraints = STADM.getDatabase().get_timing_constraint();
  constraints.get_clock_group_list().clear();
  command("set_clock_groups -asynchronous -group c");
  require(path(analyze(), "x", AnalysisType::kMax).get_start_point() == "b", "single group complement");
  constraints.get_clock_group_list().clear();
  command("set_clock_groups -asynchronous -allow_paths -group c -group d");
  require(path(analyze(), "x", AnalysisType::kMax).get_start_point() == "a", "allow_paths ignored");
  constraints.get_clock_group_list().clear();
  command("create_generated_clock -name g -source clock -master_clock c -divide_by 4 exported");
  command("set_input_delay 3 -clock g {a b}");
  command("set_output_delay 0 -clock g {x y}");
  near(path(analyze(), "x", AnalysisType::kMax).get_slack(), 35, "generated period not used in analysis");
  command("create_generated_clock -name g -source clock -master_clock c -invert exported");
  near(path(analyze(), "x", AnalysisType::kMax).get_launch_time(), 5, "generated phase not used in launch");
  near(path(analyze(), "x", AnalysisType::kMax).get_capture_time(), 15, "generated phase not used in capture");
  command("set_output_delay 0 -clock c {x y}");
  near(path(analyze(), "x", AnalysisType::kMax).get_slack(), 0, "opposite-edge setup");
  near(path(analyze(), "x", AnalysisType::kMin).get_slack(), 10, "opposite-edge hold");
}
void testGeneratedRegisterClock()
{
  fixture();
  dataGraph();
  Database& db = STADM.getDatabase();
  pin("capture:CK", false, PinDirection::kInput);
  pin("capture:D", false, PinDirection::kInput);
  pin("capture:Q", false, PinDirection::kOutput);
  Instance& instance = db.get_instance_map()["capture"];
  instance.set_is_sequential(true);
  instance.set_clock_pin_name("capture:CK");
  instance.set_output_pin_name("capture:Q");
  for (TimingCheckType type : {TimingCheckType::kSetup, TimingCheckType::kHold}) {
    TimingCheckArc check;
    check.set_clock_port("capture:CK");
    check.set_data_port("capture:D");
    check.set_check_type(type);
    instance.get_check_arc_list().push_back(check);
  }
  arc("exported", "capture:CK");
  arc("merge:Z", "capture:D");
  db.get_end_point_list() = {"capture:D"};
  db.get_timing_order_list() = {"clock", "other", "exported", "capture:CK", "a", "b", "merge:Z", "capture:D"};
  command("create_generated_clock -name g -source clock -master_clock c -divide_by 4 exported");
  command("set_input_delay 3 -clock g {a b}");
  STACP.propagate();
  require(db.get_timing_point_map().at("capture:CK").get_clock_name() == "g", "generated clock did not reach register");
  near(path(analyze(), "capture:D", AnalysisType::kMax).get_slack(), 35, "generated register setup");
  for (TimingCheckArc& check : instance.get_check_arc_list()) {
    check.set_clock_trans_type(TransType::kFall);
  }
  near(path(analyze(), "capture:D", AnalysisType::kMax).get_slack(), 15, "falling capture setup");
  near(path(analyze(), "capture:D", AnalysisType::kMin).get_slack(), 25, "falling capture hold");
  command("set_false_path -from a -to [get_pins capture/D]");
  require(path(analyze(), "capture:D", AnalysisType::kMax).get_start_point() == "b", "pin-specific exception failed");
  command("set_propagated_clock [get_clocks g]");
  setenv("ECC_LOGGER_THROW_ON_ERROR", "1", 1);
  bool rejected = false;
  try {
    STACP.propagate();
  } catch (const std::runtime_error& error) {
    rejected = std::string(error.what()).find("master-to-target insertion delay") != std::string::npos;
  }
  unsetenv("ECC_LOGGER_THROW_ON_ERROR");
  require(rejected, "propagated generated clock silently used zero insertion delay");
}

void testDefaultSdc()
{
  fixture();
  STADM.getDatabase().get_timing_constraint().get_clock_map().clear();
  command(R"(
set clk_name clock
set clk_port_name clock
set clk_freq_mhz 100.0
set clk_period [expr 1000.0 / $clk_freq_mhz]
set clk_port [get_ports $clk_port_name]
create_clock -name $clk_name -period $clk_period $clk_port
set clk_input [get_ports $clk_port_name]
set all_inputs_wo_clk [remove_from_collection [all_inputs] $clk_input]
set_input_delay 0 -clock [get_clocks $clk_name] $all_inputs_wo_clk
set_output_delay 0 -clock [get_clocks $clk_name] [all_outputs]
set_load 0.001 [all_outputs]
set clk_uncertainty [expr $clk_period * 0.05]
set clk_transition [expr min(0.15, $clk_period * 0.03)]
set input_transition [expr min(0.20, $clk_period * 0.05)]
set_clock_uncertainty $clk_uncertainty [get_clocks $clk_name]
set_clock_transition $clk_transition [get_clocks $clk_name]
set_input_transition $input_transition $all_inputs_wo_clk
set_max_fanout 20 [current_design]
)");
  TimingConstraint& constraints = STADM.getDatabase().get_timing_constraint();
  near(constraints.get_clock_map().at("clock").get_period(), 10, "SDC period");
  near(constraints.get_clock_map().at("clock").get_setup_uncertainty(), .5, "SDC uncertainty");
  near(constraints.get_port_constraint_map().at("a").get_input_transition(), .2, "SDC input slew");
  near(constraints.get_port_constraint_map().at("x").get_load(), .001, "SDC output load");
  near(*constraints.get_max_fanout(), 20, "SDC design fanout");
  require(!constraints.get_port_constraint_map().contains("clock"), "clock port received input delay");
  command("if {$all_inputs_wo_clk ne {a b other}} {error removed_wrong_ports}");
}
void testCollectionsAndValidation()
{
  fixture();
  command("set base [all_inputs]; set filtered [remove_from_collection $base {clock other}]");
  command("if {$filtered ne {a b} || $base ne {a b clock other}} {error collection_mutation}");
  command("if {[remove_from_collection -intersect $base {*o*}] ne {clock other}} {error intersection}");
  command("if {[remove_from_collection $base {}] ne $base} {error empty_removal}");
  command("if {[remove_from_collection {} $base] ne {}} {error empty_base}");
  command("if {[remove_from_collection $base missing] ne $base} {error unmatched}");
  command("if {[remove_from_collection [get_pins {sync.s_dat[0]_0/D}] {sync.s_dat[0]_0/D}] ne {}} {error bus_removal}");
  pin("name with space", true, PinDirection::kInput);
  command("if {[remove_from_collection [all_inputs] [get_ports {name with space}]] ne $base} {error spaces}", false);
  command("if {[remove_from_collection [all_inputs] [get_ports {{name with space}}]] ne $base} {error spaces}");
  pin("bus[0]", true, PinDirection::kInput);
  pin("bus[1]", true, PinDirection::kInput);
  command("if {[llength [get_ports {bus[0]}]] != 1 || [lindex [get_ports {bus[0]}] 0] ne {bus[0]}} {error bus_bit_query}");
  command("if {[llength [get_ports {bus[*]}]] != 2 || [lindex [get_ports {bus[*]}] 0] ne {bus[0]} || [lindex [get_ports {bus[*]}] 1] ne {bus[1]}} {error bus_glob_query}");
  command("if {[current_design] ne {TOP}} {error design}");
  command("current_design TOP");
  command("current_design missing", false);
  command("remove_from_collection", false);
  command("remove_from_collection -bad [all_inputs] a", false);
  command("set_clock_uncertainty .5 [get_clocks {c d}]");
  command("set_clock_uncertainty -hold .1 {c d}");
  command("set_clock_uncertainty .9 {c missing}", false);
  for (const std::string name : {"c", "d"}) {
    TimingClock& clock = STADM.getDatabase().get_timing_constraint().get_clock_map().at(name);
    near(clock.get_setup_uncertainty(), .5, "uncertainty collection setup");
    near(clock.get_hold_uncertainty(), .1, "uncertainty collection hold");
  }
  command("set_clock_transition .15 {c d}");
  command("set_clock_transition .99 {c missing}", false);
  near(STADM.getDatabase().get_timing_constraint().get_clock_map().at("c").get_transition_map()[AnalysisType::kMax][TransType::kRise], .15,
       "invalid command partially changed clocks");
  for (const std::string value : {"-1", "NaN", "Inf", "abc", "0.3junk"}) {
    command("set_clock_transition " + value + " c", false);
    command("set_max_fanout " + value + " [current_design]", false);
  }
  command("set_clock_transition .1 {}", false);
  command("set_max_fanout 3 [get_ports x]", false);
  command("set_max_fanout 3 missing", false);
  command("set_max_fanout 3 {}", false);
  command("set_max_fanout 0b101 [current_design]");
  near(*STADM.getDatabase().get_timing_constraint().get_max_fanout(), 5, "Tcl numeric value changed by option conversion");
  STADM.getDatabase().set_design_name("");
  command("if {[current_design] ne {}} {error empty_design}");
  command("set_max_fanout 20 [current_design]", false);
}
void testClockTransitionTiming()
{
  fixture();
  dataGraph();
  Database& db = STADM.getDatabase();
  pin("capture:CK", false, PinDirection::kInput);
  pin("capture:D", false, PinDirection::kInput);
  pin("capture:Q", false, PinDirection::kOutput);
  Instance& instance = db.get_instance_map()["capture"];
  instance.set_is_sequential(true);
  instance.set_clock_pin_name("capture:CK");
  instance.set_output_pin_name("capture:Q");
  TimingTable table;
  table.set_variable_type1(TimingTableVariableType::kInputTransition);
  table.set_axis_list({{0.0, 1.0}});
  table.set_value_list({0.0, 1.0});  // Check time equals clock slew.
  for (TimingCheckType type : {TimingCheckType::kSetup, TimingCheckType::kHold}) {
    TimingCheckArc check;
    check.set_clock_port("capture:CK");
    check.set_data_port("capture:D");
    check.set_check_type(type);
    TimingArc model;
    model.get_check_table_map()[TransType::kRise] = table;
    model.get_check_table_map()[TransType::kFall] = table;
    check.get_timing_arc_list().push_back(model);
    instance.get_check_arc_list().push_back(check);
  }
  arc("clock", "capture:CK");
  arc("merge:Z", "capture:D");
  db.get_end_point_list() = {"capture:D"};
  db.get_timing_order_list() = {"clock", "other", "capture:CK", "a", "b", "merge:Z", "capture:D"};
  command("set_input_delay 3 -clock c {a b}");
  STACP.propagate();
  near(path(analyze(), "capture:D", AnalysisType::kMax).get_slack(), 5, "zero-slew setup baseline");
  command("set_clock_transition .15 [get_clocks {c d}]");
  command("set_clock_transition .25 -rise -min c");
  command("set_clock_transition .35 -fall -max c");
  STACP.propagate();
  auto& slew = db.get_timing_point_map().at("capture:CK").get_clock_slew_map();
  near(slew[AnalysisType::kMin][TransType::kRise], .25, "min rise slew");
  near(slew[AnalysisType::kMax][TransType::kRise], .15, "max rise preserved");
  near(slew[AnalysisType::kMax][TransType::kFall], .35, "max fall slew");
  near(slew[AnalysisType::kMin][TransType::kFall], .15, "min fall preserved");
  near(db.get_timing_point_map().at("other").get_clock_slew_map()[AnalysisType::kMax][TransType::kRise], .15, "second clock ignored");
  near(path(analyze(), "capture:D", AnalysisType::kMax).get_slack(), 4.75, "ideal slew did not affect setup lookup");
  near(path(analyze(), "capture:D", AnalysisType::kMin).get_slack(), 4.85, "ideal slew did not affect hold lookup");
  for (TimingCheckArc& check : instance.get_check_arc_list()) {
    check.set_clock_trans_type(TransType::kFall);
  }
  near(path(analyze(), "capture:D", AnalysisType::kMax).get_slack(), -.15, "falling slew setup lookup");
  near(path(analyze(), "capture:D", AnalysisType::kMin).get_slack(), 9.65, "falling slew hold lookup");
  command("set_propagated_clock c");
  STACP.propagate();
  near(slew[AnalysisType::kMin][TransType::kRise],
       db.get_timing_point_map().at("capture:CK").get_physical_clock_slew_map()[AnalysisType::kMin][TransType::kRise],
       "ideal transition overrode propagated clock");
  command("set_clock_transition .8 c");
  STACP.propagate();
  near(slew[AnalysisType::kMin][TransType::kRise],
       db.get_timing_point_map().at("capture:CK").get_physical_clock_slew_map()[AnalysisType::kMin][TransType::kRise],
       "transition command after propagation overrode physical slew");
}
void testClockToQTransition()
{
  fixture();
  Database& db = STADM.getDatabase();
  pin("launch:CK", false, PinDirection::kInput);
  pin("launch:Q", false, PinDirection::kOutput);
  Instance& launch = db.get_instance_map()["launch"];
  launch.set_is_sequential(true);
  launch.set_clock_pin_name("launch:CK");
  launch.set_output_pin_name("launch:Q");
  TimingArc model;
  model.set_trigger_trans_type(TransType::kRise);
  model.set_sense(TimingArcSense::kNonUnate);
  TimingTable table;
  table.set_variable_type1(TimingTableVariableType::kInputTransition);
  table.set_axis_list({{0.0, 1.0}});
  table.set_value_list({1.0, 2.0});  // Clock-to-Q = 1 + clock slew.
  model.get_delay_table_map()[TransType::kRise] = table;
  model.get_delay_table_map()[TransType::kFall] = table;
  launch.get_clock_to_q_arc().get_timing_arc_list().push_back(model);
  arc("clock", "launch:CK");
  arc("launch:Q", "x");
  db.get_start_point_list() = {"launch:Q"};
  db.get_end_point_list() = {"x"};
  db.get_timing_order_list() = {"clock", "launch:CK", "launch:Q", "x"};
  command("set_output_delay 0 -clock c x");
  STACP.propagate();
  near(path(analyze(), "x", AnalysisType::kMax).get_slack(), 8.0, "clock-to-Q baseline");
  command("set_clock_transition .2 c");
  STACP.propagate();
  near(path(analyze(), "x", AnalysisType::kMax).get_slack(), 7.8, "clock-to-Q delay ignored ideal slew");
  near(path(analyze(), "x", AnalysisType::kMin).get_slack(), 2.2, "min clock-to-Q delay ignored ideal slew");
  command("set_propagated_clock c");
  STACP.propagate();
  command("set_max_delay 5 -from [get_pins launch/Q] -to x");
  const TimingPath latency_included = path(analyze(), "x", AnalysisType::kMax);
  db.get_timing_constraint().get_path_exception_list().clear();
  command("set_max_delay 5 -ignore_clock_latency -from [get_pins launch/Q] -to x");
  const TimingPath latency_ignored = path(analyze(), "x", AnalysisType::kMax);
  near(latency_ignored.get_slack() - latency_included.get_slack(), latency_included.get_launch_clock_network_delay(),
       "-ignore_clock_latency did not remove launch latency");
}

void testOutputClockUncertainty()
{
  fixture();
  dataGraph();
  command("set_input_delay 0 -clock c {a b}");
  command("set_output_delay -max 1.25 -clock c x");
  command("set_output_delay -min -.3 -clock c x");
  command("set_output_delay 0 -clock d y");
  const std::vector<TimingPath> baseline = analyze();
  near(path(baseline, "x", AnalysisType::kMax).get_required_time(), 8.75, "output setup budget");
  near(path(baseline, "x", AnalysisType::kMin).get_required_time(), .3, "negative output hold delay");
  command("set_clock_uncertainty -setup .7 c");
  command("set_clock_uncertainty -hold .2 c");
  command("set_clock_uncertainty -setup .9 d");
  command("set_clock_uncertainty -hold .4 d");
  const std::vector<TimingPath> constrained = analyze();
  near(path(constrained, "x", AnalysisType::kMax).get_required_time(), 8.05, "output setup uncertainty");
  near(path(constrained, "x", AnalysisType::kMin).get_required_time(), .5, "output hold uncertainty");
  near(path(constrained, "y", AnalysisType::kMax).get_slack(), path(baseline, "y", AnalysisType::kMax).get_slack() - .9,
       "output setup uncertainty must use capture clock");
  near(path(constrained, "y", AnalysisType::kMin).get_slack(), path(baseline, "y", AnalysisType::kMin).get_slack() - .4,
       "output hold uncertainty must use capture clock");

  const std::filesystem::path directory = std::filesystem::temp_directory_path() / "ista_sdc_output_uncertainty_test";
  std::filesystem::create_directories(directory);
  const auto saved_config = STADM.getConfig();
  STADM.getConfig().tr_temp_directory_path = directory.string() + "/";
  STADM.getConfig().output_timing_reports = 1;
  STADM.getConfig().has_timing_report_slack_lesser_than = true;
  STADM.getConfig().timing_report_slack_lesser_than = 1000;
  TimingReporter::initInst();
  STATR.report();
  for (const std::string mode : {"max", "min"}) {
    std::ifstream report(directory / ("timing_" + mode + "_in2out.rpt"));
    std::string line;
    bool selected = false;
    int checked = 0;
    while (std::getline(report, line)) {
      if (line.find("Endpoint:") != std::string::npos) {
        selected = line.find("Endpoint: x") != std::string::npos;
      }
      for (const std::string label : {"clock uncertainty", "output external delay"}) {
        const std::size_t position = line.find(label);
        if (!selected || position == std::string::npos) {
          continue;
        }
        std::istringstream values(line.substr(position + label.size()));
        double increment = 0.0;
        double cumulative = 0.0;
        require(static_cast<bool>(values >> increment >> cumulative), "missing output report columns");
        const bool uncertainty = label == "clock uncertainty";
        near(increment, mode == "max" ? (uncertainty ? -.7 : -1.25) : (uncertainty ? .2 : .3), "output report increment");
        near(cumulative, mode == "max" ? (uncertainty ? 9.3 : 8.05) : (uncertainty ? .2 : .5), "output report cumulative time");
        ++checked;
      }
    }
    require(checked >= 2, "missing output uncertainty/delay report rows");
  }
  STADM.getConfig() = saved_config;
  command("set_output_delay 0 -clock c x");
  near(path(analyze(), "x", AnalysisType::kMin).get_slack(), 1.8, "zero output delay still uses uncertainty");
  command("set_clock_uncertainty -hold 2.5 c");
  near(path(analyze(), "x", AnalysisType::kMin).get_slack(), -.5, "output uncertainty must create hold violations");
  command("set_clock_uncertainty 0 c");
  near(path(analyze(), "x", AnalysisType::kMin).get_slack(), 2.0, "clearing output uncertainty restores timing");
}

void testFanoutAnalysisAndReports()
{
  fixture();
  dataGraph();
  Database& db = STADM.getDatabase();
  TimingCell& cell = db.get_timing_library().get_cell_map()["sink_cell"];
  cell.get_port_map()["A"].set_fanout_load(1.5);
  cell.get_port_map()["B"].set_fanout_load(2.0);
  for (const std::string name : {"sink:A", "sink:B"}) {
    pin(name, false, PinDirection::kInput);
  }
  db.get_instance_map()["sink"].set_cell_name("sink_cell");
  Net& net = db.get_net_map()["inputs"];
  net.set_driver_pin("a");
  net.set_driver_pin_list({"a"});
  net.set_load_pin_list({"sink:A", "sink:B", "x"});
  command("set_max_fanout 3 [current_design]");
  command("set_max_fanout 4 a");
  const double slack_before = path(analyze(), "x", AnalysisType::kMax).get_slack();
  require(db.get_fanout_check_list().size() == 1, "duplicate driver counted twice");
  near(db.get_fanout_check_list().front().get_slack(), -.5, "weighted fanout/design precedence");
  command("set_max_fanout 2 a");
  command("set_max_fanout 1 {a x}", false);
  near(path(analyze(), "x", AnalysisType::kMax).get_slack(), slack_before, "fanout constraint changed path delay");
  near(db.get_fanout_check_list().front().get_slack(), -1.5, "port precedence/atomic failure");
  pin("driver:Z", false, PinDirection::kOutput);
  db.get_instance_map()["driver"].set_cell_name("driver_cell");
  TimingCellPort& driver_port = db.get_timing_library().get_cell_map()["driver_cell"].get_port_map()["Z"];
  driver_port.set_max_fanout(1.0);
  Net& internal_net = db.get_net_map()["internal"];
  internal_net.set_driver_pin("driver:Z");
  internal_net.set_load_pin_list({"sink:B"});
  analyze();
  require(db.get_fanout_check_list().size() == 2, "missing library fanout check");
  near(db.get_fanout_check_list().back().get_slack(), -1.0, "pin library limit");
  command("set_max_fanout 3.5 a");
  command("set_max_fanout 3.5 [current_design]");
  analyze();
  require(db.get_fanout_check_list().size() == 2, "fanout checks accumulated across runs");
  near(db.get_fanout_check_list().back().get_slack(), 0.0, "equality is not a violation");
  const std::filesystem::path directory = std::filesystem::temp_directory_path() / "ista_sdc_fanout_test";
  std::filesystem::create_directories(directory);
  STADM.getConfig().tr_temp_directory_path = directory.string() + "/";
  STADM.getConfig().output_timing_reports = 1;
  STADM.getConfig().output_timing_features = 1;
  TimingReporter::initInst();
  STATR.report();
  std::ifstream report(directory / "max_fanout.rpt");
  std::string report_text((std::istreambuf_iterator<char>(report)), std::istreambuf_iterator<char>());
  require(report_text.find("Checked drivers: 2; violations: 1") != std::string::npos, "fanout report missing results");
  std::ifstream qor(directory / "qor_summary.json");
  std::string qor_text((std::istreambuf_iterator<char>(qor)), std::istreambuf_iterator<char>());
  require(qor_text.find("\"fanout\":1") != std::string::npos, "QoR fanout remains hardcoded");
  db.get_timing_constraint().get_max_fanout().reset();
  db.get_timing_constraint().get_port_max_fanout_map().clear();
  driver_port.get_max_fanout().reset();
  analyze();
  require(db.get_fanout_check_list().empty(), "stale fanout checks after clearing limits");
}
}  // namespace

int main()
{
  Logger::initInst();
  DataManager::initInst();
  DelayCalculator::initInst();
  TimingAnalyzer::initInst();
  TimingPropagator::initInst();
  ClockPropagator::initInst();
  SdcCommand::initInst({
      {"current_design", sdc::executeTclCommand<sdc::TclCurrentDesign>},
      {"remove_from_collection", sdc::executeTclCommand<sdc::TclRemoveFromCollection>},
      {"set_clock_transition", sdc::executeTclCommand<sdc::TclSetClockTransition>},
      {"set_max_fanout", sdc::executeTclCommand<sdc::TclSetMaxFanout>},
      {"set_clock_uncertainty", sdc::executeTclCommand<sdc::TclSetClockUncertainty>},
      {"set_input_transition", sdc::executeTclCommand<sdc::TclSetInputTransition>},
      {"set_load", sdc::executeTclCommand<sdc::TclSetLoad>},
      {"create_clock", sdc::executeTclCommand<sdc::TclCreateClock>},
      {"create_generated_clock", sdc::executeTclCommand<sdc::TclCreateGeneratedClock>},
      {"get_ports", sdc::executeTclCommand<sdc::TclGetPorts>},
      {"get_pins", sdc::executeTclCommand<sdc::TclGetPins>},
      {"get_cells", sdc::executeTclCommand<sdc::TclGetCells>},
      {"get_nets", sdc::executeTclCommand<sdc::TclGetNets>},
      {"get_clocks", sdc::executeTclCommand<sdc::TclGetClocks>},
      {"all_inputs", sdc::executeTclCommand<sdc::TclAllInputs>},
      {"all_outputs", sdc::executeTclCommand<sdc::TclAllOutputs>},
      {"set_input_delay", sdc::executeTclCommand<sdc::TclSetInputDelay>},
      {"set_output_delay", sdc::executeTclCommand<sdc::TclSetOutputDelay>},
      {"set_false_path", sdc::executeTclCommand<sdc::TclSetFalsePath>},
      {"set_max_delay", sdc::executeTclCommand<sdc::TclSetMaxDelay>},
      {"set_min_delay", sdc::executeTclCommand<sdc::TclSetMinDelay>},
      {"set_multicycle_path", sdc::executeTclCommand<sdc::TclSetMulticyclePath>},
      {"reset_path", sdc::executeTclCommand<sdc::TclResetPath>},
      {"unset_path_exceptions", sdc::executeTclCommand<sdc::TclResetPath>},
      {"set_clock_groups", sdc::executeTclCommand<sdc::TclSetClockGroups>},
      {"set_propagated_clock", sdc::executeTclCommand<sdc::TclSetPropagatedClock>},
  });
  try {
    testKivenEnvironmentConstraints();
    testCaseAnalysisNoopWithoutConstraints();
    testQueriesAndGeneratedClocks();
    testExceptionsPreserveAlternatives();
    testOrderedFalsePathExceptions();
    testPathDelayAndMulticycleExceptions();
    testClockGroupsAndWaveforms();
    testGeneratedRegisterClock();
    testDefaultSdc();
    testCollectionsAndValidation();
    testClockTransitionTiming();
    testClockToQTransition();
    testOutputClockUncertainty();
    testFanoutAnalysisAndReports();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << "SDC queries, generated clocks, timing exceptions, edge pairing: PASS\n";
  return 0;
}

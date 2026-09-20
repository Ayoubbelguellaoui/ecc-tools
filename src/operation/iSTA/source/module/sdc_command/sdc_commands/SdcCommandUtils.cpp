// ***************************************************************************************
// Copyright (c) 2023-2025 Peng Cheng Laboratory
// Copyright (c) 2023-2025 Institute of Computing Technology, Chinese Academy of Sciences
// Copyright (c) 2023-2025 Beijing Institute of Open Source Chip
//
// iEDA is licensed under Mulan PSL v2.
// You can use this software according to the terms and conditions of the Mulan PSL v2.
// You may obtain a copy of Mulan PSL v2 at:
// http://license.coscl.org.cn/MulanPSL2
//
// THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
// EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
// MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
//
// See the Mulan PSL v2 for more details.
// ***************************************************************************************
#include "SdcCommandUtils.hpp"

#include "STAHeader.hpp"
#include "SdcCommand.hpp"
#include "SdcTclCmd.hpp"

namespace ista::sdc {

namespace {

struct QueryCandidate
{
  std::string canonical_name;
  std::vector<std::string> match_names;
  std::map<std::string, std::string> attributes;
};

std::string trim(std::string text)
{
  const std::size_t first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const std::size_t last = text.find_last_not_of(" \t\r\n");
  text = text.substr(first, last - first + 1);
  while (text.size() >= 2 && ((text.front() == '{' && text.back() == '}') || (text.front() == '(' && text.back() == ')'))) {
    int depth = 0;
    bool encloses_all = true;
    for (std::size_t index = 0; index < text.size(); ++index) {
      if (text[index] == text.front()) ++depth;
      if (text[index] == text.back()) --depth;
      if (depth == 0 && index + 1 < text.size()) {
        encloses_all = false;
        break;
      }
    }
    if (!encloses_all) break;
    text = trim(text.substr(1, text.size() - 2));
  }
  if (text.size() >= 2 && ((text.front() == '"' && text.back() == '"') || (text.front() == '\'' && text.back() == '\''))) {
    text = text.substr(1, text.size() - 2);
  }
  return text;
}

std::size_t findTopLevelOperator(const std::string& expression, std::string_view operation)
{
  int parentheses = 0;
  int braces = 0;
  char quote = '\0';
  for (std::size_t index = 0; index + operation.size() <= expression.size(); ++index) {
    const char value = expression[index];
    if (quote != '\0') {
      if (value == quote && (index == 0 || expression[index - 1] != '\\')) quote = '\0';
      continue;
    }
    if (value == '"' || value == '\'') {
      quote = value;
      continue;
    }
    if (value == '(') ++parentheses;
    if (value == ')') --parentheses;
    if (value == '{') ++braces;
    if (value == '}') --braces;
    if (parentheses == 0 && braces == 0 && expression.compare(index, operation.size(), operation) == 0) return index;
  }
  return std::string::npos;
}

bool asNumber(const std::string& text, double& value)
{
  char* end = nullptr;
  value = std::strtod(text.c_str(), &end);
  return end != text.c_str() && *end == '\0' && std::isfinite(value);
}

bool evaluateFilter(const std::string& raw_expression, const std::map<std::string, std::string>& attributes)
{
  const std::string expression = trim(raw_expression);
  if (expression.empty()) return true;
  for (std::string_view operation : {std::string_view("||"), std::string_view("&&")}) {
    const std::size_t position = findTopLevelOperator(expression, operation);
    if (position != std::string::npos) {
      const bool left = evaluateFilter(expression.substr(0, position), attributes);
      const bool right = evaluateFilter(expression.substr(position + operation.size()), attributes);
      return operation == "||" ? left || right : left && right;
    }
  }
  if (expression.front() == '!') return !evaluateFilter(expression.substr(1), attributes);

  for (std::string_view operation : {std::string_view("!~"), std::string_view("=~"), std::string_view("!="), std::string_view("=="),
                                     std::string_view(">="), std::string_view("<="), std::string_view(">"), std::string_view("<")}) {
    const std::size_t position = findTopLevelOperator(expression, operation);
    if (position == std::string::npos) continue;
    const std::string attribute = trim(expression.substr(0, position));
    const std::string expected = trim(expression.substr(position + operation.size()));
    const auto actual_iter = attributes.find(attribute);
    if (actual_iter == attributes.end()) return false;
    const std::string& actual = actual_iter->second;
    if (operation == "=~" || operation == "!~") {
      const bool match = Tcl_StringMatch(actual.c_str(), expected.c_str()) != 0;
      return operation == "=~" ? match : !match;
    }
    if (operation == "==" || operation == "!=") {
      const bool match = actual == expected;
      return operation == "==" ? match : !match;
    }
    double left = 0.0;
    double right = 0.0;
    if (!asNumber(actual, left) || !asNumber(expected, right)) return false;
    if (operation == ">=") return left >= right;
    if (operation == "<=") return left <= right;
    if (operation == ">") return left > right;
    return left < right;
  }
  const auto attribute = attributes.find(expression);
  return attribute != attributes.end() && attribute->second != "false" && attribute->second != "0" && !attribute->second.empty();
}

std::string directionName(PinDirection direction)
{
  switch (direction) {
    case PinDirection::kInput:
      return "in";
    case PinDirection::kOutput:
      return "out";
    case PinDirection::kInout:
      return "inout";
    default:
      return "internal";
  }
}

std::string leafName(std::string name)
{
  const std::size_t separator = name.find_last_of("/:");
  return separator == std::string::npos ? name : name.substr(separator + 1);
}

void addCandidate(std::vector<QueryCandidate>& candidates, std::string canonical, std::string display, std::string object_class,
                  std::map<std::string, std::string> attributes = {})
{
  QueryCandidate candidate;
  candidate.canonical_name = std::move(canonical);
  candidate.match_names = {candidate.canonical_name};
  if (display != candidate.canonical_name) candidate.match_names.push_back(display);
  candidate.match_names.push_back(leafName(display));
  attributes["name"] = leafName(display);
  attributes["full_name"] = display;
  attributes["object_class"] = std::move(object_class);
  candidate.attributes = std::move(attributes);
  candidates.push_back(std::move(candidate));
}

bool hasUnescapedBracket(const std::string& pattern)
{
  for (std::size_t index = 0; index < pattern.size(); ++index) {
    if ((pattern[index] == '[' || pattern[index] == ']') && (index == 0 || pattern[index - 1] != '\\')) return true;
  }
  return false;
}

std::string escapeGlobBrackets(const std::string& pattern)
{
  std::string escaped;
  escaped.reserve(pattern.size() + 4);
  for (std::size_t index = 0; index < pattern.size(); ++index) {
    if ((pattern[index] == '[' || pattern[index] == ']') && (index == 0 || pattern[index - 1] != '\\')) escaped.push_back('\\');
    escaped.push_back(pattern[index]);
  }
  return escaped;
}

std::vector<QueryCandidate> buildCandidates(Database& database, QueryObjectType type)
{
  std::vector<QueryCandidate> candidates;
  if (type == QueryObjectType::kClock || type == QueryObjectType::kAny) {
    for (auto& [name, clock] : database.get_timing_constraint().get_clock_map()) {
      addCandidate(candidates, name, name, "clock", {{"period", std::to_string(clock.get_period())},
                                                       {"is_generated", clock.get_is_generated() ? "true" : "false"},
                                                       {"is_propagated", clock.get_is_propagated() ? "true" : "false"}});
    }
  }
  if (type == QueryObjectType::kPort || type == QueryObjectType::kPin || type == QueryObjectType::kAny) {
    for (auto& [name, pin] : database.get_pin_map()) {
      if ((type == QueryObjectType::kPort && !pin.get_is_port()) || (type == QueryObjectType::kPin && pin.get_is_port())) continue;
      const std::string display = pin.get_is_port() ? name : pin.get_instance_name() + "/" + pin.get_pin_name();
      std::map<std::string, std::string> attributes{{"direction", directionName(pin.get_direction())},
                                                    {"is_port", pin.get_is_port() ? "true" : "false"}};
      if (!pin.get_is_port()) {
        attributes["pin_name"] = pin.get_pin_name();
        const auto instance = database.get_instance_map().find(pin.get_instance_name());
        if (instance != database.get_instance_map().end()) attributes["ref_name"] = instance->second.get_cell_name();
      }
      addCandidate(candidates, name, display, pin.get_is_port() ? "port" : "pin", std::move(attributes));
    }
  }
  if (type == QueryObjectType::kCell || type == QueryObjectType::kAny) {
    for (auto& [name, instance] : database.get_instance_map()) {
      addCandidate(candidates, name, name, "cell", {{"ref_name", instance.get_cell_name()},
                                                     {"is_sequential", instance.get_is_sequential() ? "true" : "false"},
                                                     {"is_clock_gating_cell", instance.get_is_clock_gating() ? "true" : "false"}});
    }
  }
  if (type == QueryObjectType::kNet || type == QueryObjectType::kAny) {
    for (auto& [name, net] : database.get_net_map()) {
      addCandidate(candidates, name, name, "net", {{"fanout", std::to_string(net.get_load_pin_list().size())}});
    }
  }
  return candidates;
}

QueryObjectType objectType(Database& database, const std::string& name)
{
  const auto pin = database.get_pin_map().find(name);
  if (pin != database.get_pin_map().end()) return pin->second.get_is_port() ? QueryObjectType::kPort : QueryObjectType::kPin;
  if (database.get_timing_constraint().get_clock_map().contains(name)) return QueryObjectType::kClock;
  if (database.get_instance_map().contains(name)) return QueryObjectType::kCell;
  if (database.get_net_map().contains(name)) return QueryObjectType::kNet;
  return QueryObjectType::kAny;
}

std::set<std::string> objectsOf(Database& database, const std::vector<std::string>& objects, QueryObjectType target)
{
  std::set<std::string> result;
  for (const std::string& raw : objects) {
    std::vector<std::string> resolved = queryObjects(database, {raw}, QueryObjectType::kAny);
    for (const std::string& object : resolved) {
      const QueryObjectType source = objectType(database, object);
      if (source == target) result.insert(object);
      if (source == QueryObjectType::kPin || source == QueryObjectType::kPort) {
        Pin& pin = database.get_pin_map().at(object);
        if (target == QueryObjectType::kCell && !pin.get_instance_name().empty()) result.insert(pin.get_instance_name());
        if (target == QueryObjectType::kNet && !pin.get_net_name().empty()) result.insert(pin.get_net_name());
      }
      if (source == QueryObjectType::kCell) {
        for (const std::string& pin_name : database.get_instance_map().at(object).get_pin_name_list()) {
          if (target == QueryObjectType::kPin) result.insert(pin_name);
          if (target == QueryObjectType::kNet && database.get_pin_map().contains(pin_name)
              && !database.get_pin_map().at(pin_name).get_net_name().empty()) {
            result.insert(database.get_pin_map().at(pin_name).get_net_name());
          }
        }
      }
      if (source == QueryObjectType::kNet) {
        Net& net = database.get_net_map().at(object);
        for (const std::string& pin_name : net.get_pin_name_list()) {
          if (target == QueryObjectType::kPin && database.get_pin_map().contains(pin_name) && !database.get_pin_map().at(pin_name).get_is_port()) {
            result.insert(pin_name);
          }
          if (target == QueryObjectType::kPort && database.get_pin_map().contains(pin_name) && database.get_pin_map().at(pin_name).get_is_port()) {
            result.insert(pin_name);
          }
          if (target == QueryObjectType::kCell && database.get_pin_map().contains(pin_name)
              && !database.get_pin_map().at(pin_name).get_instance_name().empty()) {
            result.insert(database.get_pin_map().at(pin_name).get_instance_name());
          }
        }
      }
      if (target == QueryObjectType::kClock && (source == QueryObjectType::kPin || source == QueryObjectType::kPort)) {
        for (auto& [clock_name, clock] : database.get_timing_constraint().get_clock_map()) {
          if (std::find(clock.get_source_list().begin(), clock.get_source_list().end(), object) != clock.get_source_list().end()) result.insert(clock_name);
        }
      }
    }
  }
  return result;
}

bool textMatches(std::string actual, std::string pattern, const QueryOptions& options, bool escape_brackets)
{
  if (options.nocase) {
    std::transform(actual.begin(), actual.end(), actual.begin(), [](unsigned char value) { return std::tolower(value); });
    std::transform(pattern.begin(), pattern.end(), pattern.begin(), [](unsigned char value) { return std::tolower(value); });
  }
  if (options.exact) return actual == pattern;
  if (options.regexp) {
    const std::string expression = "^(?:" + pattern + ")$";
    if (Tcl_RegExpMatch(SdcCommand::getInst().getInterp(), "", expression.c_str()) < 0) throw std::invalid_argument("invalid regular expression: " + pattern);
    return Tcl_RegExpMatch(SdcCommand::getInst().getInterp(), actual.c_str(), expression.c_str()) == 1;
  }
  if (escape_brackets) pattern = escapeGlobBrackets(pattern);
  return Tcl_StringMatch(actual.c_str(), pattern.c_str()) != 0;
}

}  // namespace

std::vector<std::string> queryPatterns(const std::string& text, bool regexp)
{
  // OpenSTA SDC commonly passes one raw regexp; PT writers pass a Tcl list.
  if (regexp && !text.empty() && text.front() != '{' && text.find('\\') != std::string::npos) {
    return {text};
  }
  int count = 0;
  const char** items = nullptr;
  if (Tcl_SplitList(SdcCommand::getInst().getInterp(), text.c_str(), &count, &items) != TCL_OK) {
    throw std::invalid_argument("invalid object pattern list");
  }
  std::vector<std::string> result(items, items + count);
  Tcl_Free(reinterpret_cast<char*>(items));
  return result;
}

std::vector<std::string> queryObjects(Database& database, const std::vector<std::string>& patterns, QueryObjectType type, bool regexp)
{
  QueryOptions options;
  options.regexp = regexp;
  return queryObjects(database, patterns, type, options);
}

std::vector<std::string> queryObjects(Database& database, const std::vector<std::string>& patterns, QueryObjectType type, const QueryOptions& options)
{
  if (options.regexp && options.exact) throw std::invalid_argument("-regexp and -exact are mutually exclusive");
  const std::set<std::string> of_objects = options.of_objects.empty() ? std::set<std::string>{} : objectsOf(database, options.of_objects, type);
  const std::vector<QueryCandidate> candidates = buildCandidates(database, type);
  std::set<std::string> found;

  auto isEligible = [&](const QueryCandidate& candidate) {
    return (options.of_objects.empty() || of_objects.contains(candidate.canonical_name)) && evaluateFilter(options.filter, candidate.attributes);
  };
  auto matches = [&](const QueryCandidate& candidate, const std::string& pattern, bool escape_brackets) {
    for (std::size_t index = 0; index < candidate.match_names.size(); ++index) {
      if (index == 2 && !options.hierarchical) continue;
      if (textMatches(candidate.match_names[index], pattern, options, escape_brackets)) return true;
    }
    return false;
  };

  for (const std::string& pattern : patterns) {
    bool pattern_matched = false;
    for (const QueryCandidate& candidate : candidates) {
      if (isEligible(candidate) && matches(candidate, pattern, false)) {
        found.insert(candidate.canonical_name);
        pattern_matched = true;
      }
    }

    // OpenSTA retries a non-regexp query with literal brackets when the normal
    // glob pass found nothing. This is required for names such as bus[0].
    if (!pattern_matched && !options.regexp && !options.exact && hasUnescapedBracket(pattern)) {
      for (const QueryCandidate& candidate : candidates) {
        if (isEligible(candidate) && matches(candidate, pattern, true)) {
          found.insert(candidate.canonical_name);
        }
      }
    }
  }
  return {found.begin(), found.end()};
}

std::vector<std::string> resolveTypedObjects(Database& database, const std::vector<std::string>& objects, QueryObjectType type)
{
  std::set<std::string> resolved;
  for (const std::string& object : objects) {
    const std::vector<std::string> matches = queryObjects(database, {object}, type);
    resolved.insert(matches.begin(), matches.end());
  }
  return {resolved.begin(), resolved.end()};
}

std::vector<std::string> resolveClockSources(Database& database, const std::vector<std::string>& objects)
{
  std::set<std::string> sources;
  for (const std::string& object : objects) {
    for (const std::string& match : queryObjects(database, {object}, QueryObjectType::kAny)) {
      const QueryObjectType type = objectType(database, match);
      if (type == QueryObjectType::kPort || type == QueryObjectType::kPin) {
        sources.insert(match);
      } else if (type == QueryObjectType::kNet) {
        Net& net = database.get_net_map().at(match);
        if (!net.get_driver_pin().empty()) sources.insert(net.get_driver_pin());
        sources.insert(net.get_driver_pin_list().begin(), net.get_driver_pin_list().end());
      } else {
        throw std::invalid_argument("clock sources must be ports, pins, or nets: " + object);
      }
    }
  }
  return {sources.begin(), sources.end()};
}

std::set<std::string> resolveClockObjects(Database& database, const std::vector<std::string>& objects)
{
  std::set<std::string> clocks;
  for (const std::string& pattern : objects) {
    const std::vector<std::string> matches = queryObjects(database, {pattern}, QueryObjectType::kClock);
    if (matches.empty()) {
      throw std::invalid_argument("clock '" + pattern + "' does not exist");
    }
    clocks.insert(matches.begin(), matches.end());
  }
  if (clocks.empty()) {
    throw std::invalid_argument("a non-empty clock collection is required");
  }
  return clocks;
}

std::set<std::string> resolveExceptionObjects(Database& database, const std::vector<std::string>& objects)
{
  std::set<std::string> result;
  for (const std::string& object : objects) {
    if (database.get_pin_map().contains(object) || database.get_instance_map().contains(object) || database.get_net_map().contains(object)
        || database.get_timing_constraint().get_clock_map().contains(object)) {
      result.insert(object);
      continue;
    }
    const std::vector<std::string> matches = queryObjects(database, {object}, QueryObjectType::kAny);
    if (matches.empty()) {
      throw std::invalid_argument("exception object not found: " + object);
    }
    result.insert(matches.begin(), matches.end());
  }
  if (result.empty()) {
    throw std::invalid_argument("empty timing exception collection");
  }
  return result;
}

std::vector<std::string> resolveObjectList(Database& database, const std::vector<std::string>& object_list)
{
  std::vector<std::string> resolved_object_list;
  for (const std::string& object_name : object_list) {
    std::string resolved_object_name = object_name;
    if (!resolved_object_name.empty() && resolved_object_name.front() == '\\') {
      resolved_object_name.erase(resolved_object_name.begin());
    }
    if (resolved_object_name.rfind("[get_ports", 0) == 0) {
      resolved_object_name = resolved_object_name.substr(10);
    }
    if (resolved_object_name.rfind("[get_pins", 0) == 0) {
      resolved_object_name = resolved_object_name.substr(9);
      std::replace(resolved_object_name.begin(), resolved_object_name.end(), '/', ':');
    }
    if (database.get_pin_map().count(resolved_object_name) > 0) {
      resolved_object_list.push_back(resolved_object_name);
      continue;
    }
    if (!resolved_object_name.empty() && resolved_object_name.back() == ']') {
      std::string trimmed_object_name = resolved_object_name;
      trimmed_object_name.pop_back();
      if (database.get_pin_map().count(trimmed_object_name) > 0) {
        resolved_object_list.push_back(trimmed_object_name);
        continue;
      }
    }
    std::replace(resolved_object_name.begin(), resolved_object_name.end(), '/', ':');
    if (database.get_pin_map().count(resolved_object_name) > 0) {
      resolved_object_list.push_back(resolved_object_name);
    }
  }
  return resolved_object_list;
}

TimingPortConstraint& getPortConstraint(Database& database, const std::string& port_name)
{
  TimingPortConstraint& port_constraint = database.get_timing_constraint().get_port_constraint_map()[port_name];
  port_constraint.set_port_name(port_name);
  return port_constraint;
}

void addQueryOptions(SdcTclCmd& command, bool hierarchical, bool exact, bool of_objects)
{
  command.addOption(new ecc::TclSwitchOption("-quiet"));
  command.addOption(new ecc::TclSwitchOption("-regexp"));
  command.addOption(new ecc::TclSwitchOption("-nocase"));
  command.addOption(new ecc::TclStringOption("-filter", 0));
  if (hierarchical) command.addOption(new ecc::TclSwitchOption("-hierarchical"));
  if (exact) command.addOption(new ecc::TclSwitchOption("-exact"));
  if (of_objects) command.addOption(new ecc::TclStringListOption("-of_objects", 0));
}

QueryOptions getQueryOptions(SdcTclCmd& command)
{
  QueryOptions options;
  for (const auto& [name, target] : std::initializer_list<std::pair<const char*, bool*>>{{"-regexp", &options.regexp},
                                                                                        {"-nocase", &options.nocase},
                                                                                        {"-exact", &options.exact},
                                                                                        {"-hierarchical", &options.hierarchical}}) {
    if (ecc::TclOption* option = command.getOptionOrArg(name); option != nullptr) *target = option->is_set_val();
  }
  if (ecc::TclOption* filter = command.getOptionOrArg("-filter"); filter != nullptr && filter->is_set_val()) options.filter = filter->getStringVal();
  if (ecc::TclOption* objects = command.getOptionOrArg("-of_objects"); objects != nullptr && objects->is_set_val()) {
    options.of_objects = objects->getStringList();
  }
  return options;
}

}  // namespace ista::sdc

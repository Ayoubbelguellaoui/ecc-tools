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
#include "DataManager.hpp"
#include "SdcCommandUtils.hpp"
#include "SdcCommands.hpp"

namespace ista::sdc {

TclSetFalsePath::TclSetFalsePath(const char* cmd_name, ClientData client_data) : SdcTclCmd(cmd_name, client_data)
{
  addOption(new ecc::TclStringListOption("-from", 0));
  addOption(new ecc::TclStringListOption("-rise_from", 0));
  addOption(new ecc::TclStringListOption("-fall_from", 0));
  addOption(new ecc::TclStringListListOption("-through", 0));
  addOption(new ecc::TclStringListListOption("-rise_through", 0));
  addOption(new ecc::TclStringListListOption("-fall_through", 0));
  addOption(new ecc::TclStringListOption("-to", 0));
  addOption(new ecc::TclStringListOption("-rise_to", 0));
  addOption(new ecc::TclStringListOption("-fall_to", 0));
  addOption(new ecc::TclSwitchOption("-setup"));
  addOption(new ecc::TclSwitchOption("-hold"));
  addOption(new ecc::TclSwitchOption("-rise"));
  addOption(new ecc::TclSwitchOption("-fall"));
  addOption(new ecc::TclSwitchOption("-reset_path"));
  addOption(new ecc::TclStringOption("-comment", 0));
}

unsigned TclSetFalsePath::exec()
{
  int from_count = 0;
  int to_count = 0;
  for (const char* option : {"-from", "-rise_from", "-fall_from"}) {
    from_count += getOptionOrArg(option)->is_set_val();
  }
  for (const char* option : {"-to", "-rise_to", "-fall_to"}) {
    to_count += getOptionOrArg(option)->is_set_val();
  }
  if (from_count > 1 || to_count > 1) {
    setTclError("set_false_path accepts only one from selector and one to selector");
    return 0;
  }
  const bool has_through = getOptionOrArg("-through")->is_set_val() || getOptionOrArg("-rise_through")->is_set_val()
                           || getOptionOrArg("-fall_through")->is_set_val();
  if (from_count == 0 && to_count == 0 && !has_through) {
    setTclError("set_false_path requires -from, -through, or -to");
    return 0;
  }

  Database& database = STADM.getDatabase();
  TimingException exception;
  const bool setup = getOptionOrArg("-setup")->is_set_val();
  const bool hold = getOptionOrArg("-hold")->is_set_val();
  const bool rise = getOptionOrArg("-rise")->is_set_val();
  const bool fall = getOptionOrArg("-fall")->is_set_val();
  exception.set_setup(setup || !hold);
  exception.set_hold(hold || !setup);
  exception.set_rise(rise || !fall);
  exception.set_fall(fall || !rise);

  for (const std::pair<const char*, TransType>& selector : {std::pair{"-from", TransType::kNone}, std::pair{"-rise_from", TransType::kRise},
                                                            std::pair{"-fall_from", TransType::kFall}}) {
    if (getOptionOrArg(selector.first)->is_set_val()) {
      exception.set_from_objects(resolveExceptionObjects(database, getOptionOrArg(selector.first)->getStringList()));
      exception.set_from_trans_type(selector.second);
    }
  }
  for (const std::pair<const char*, TransType>& selector : {std::pair{"-to", TransType::kNone}, std::pair{"-rise_to", TransType::kRise},
                                                            std::pair{"-fall_to", TransType::kFall}}) {
    if (getOptionOrArg(selector.first)->is_set_val()) {
      exception.set_to_objects(resolveExceptionObjects(database, getOptionOrArg(selector.first)->getStringList()));
      exception.set_to_trans_type(selector.second);
    }
  }

  std::vector<TimingExceptionThrough> through_list;
  for (const std::pair<std::string, std::string>& option_value : getOptionValueList()) {
    TransType trans_type = TransType::kNone;
    if (option_value.first == "-rise_through") {
      trans_type = TransType::kRise;
    } else if (option_value.first == "-fall_through") {
      trans_type = TransType::kFall;
    } else if (option_value.first != "-through") {
      continue;
    }
    TimingExceptionThrough through;
    through.set_objects(resolveExceptionObjects(database, queryPatterns(option_value.second, false)));
    through.set_trans_type(trans_type);
    through_list.push_back(std::move(through));
  }
  exception.set_through_list(through_list);
  if (getOptionOrArg("-comment")->is_set_val()) {
    exception.set_comment(getOptionOrArg("-comment")->getStringVal());
  }

  std::vector<TimingException>& false_paths = database.get_timing_constraint().get_false_path_list();
  if (getOptionOrArg("-reset_path")->is_set_val()) {
    for (TimingException& existing : false_paths) {
      bool same_through = existing.get_through_list().size() == exception.get_through_list().size();
      for (std::size_t index = 0; same_through && index < existing.get_through_list().size(); ++index) {
        same_through = existing.get_through_list()[index].get_objects() == exception.get_through_list()[index].get_objects()
                       && existing.get_through_list()[index].get_trans_type() == exception.get_through_list()[index].get_trans_type();
      }
      if (existing.get_from_objects() == exception.get_from_objects() && existing.get_to_objects() == exception.get_to_objects()
          && existing.get_from_trans_type() == exception.get_from_trans_type() && existing.get_to_trans_type() == exception.get_to_trans_type()
          && existing.get_rise() == exception.get_rise() && existing.get_fall() == exception.get_fall() && same_through) {
        if (exception.get_setup()) {
          existing.set_setup(false);
        }
        if (exception.get_hold()) {
          existing.set_hold(false);
        }
      }
    }
    std::erase_if(false_paths, [](const TimingException& existing) { return !existing.get_setup() && !existing.get_hold(); });
  }
  false_paths.push_back(std::move(exception));
  return 1;
}

}  // namespace ista::sdc

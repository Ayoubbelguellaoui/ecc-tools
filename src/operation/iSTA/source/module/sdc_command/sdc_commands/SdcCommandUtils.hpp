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
#pragma once

#include "Database.hpp"
#include "STAHeader.hpp"
namespace ista::sdc {

class SdcTclCmd;

enum class QueryObjectType
{
  kPort,
  kPin,
  kCell,
  kNet,
  kClock,
  kAny
};

struct QueryOptions
{
  bool regexp = false;
  bool nocase = false;
  bool exact = false;
  bool hierarchical = false;
  std::string filter;
  std::vector<std::string> of_objects;
};

std::vector<std::string> queryPatterns(const std::string& text, bool regexp);
std::vector<std::string> queryObjects(Database& database, const std::vector<std::string>& patterns, QueryObjectType type, bool regexp = false);
std::vector<std::string> queryObjects(Database& database, const std::vector<std::string>& patterns, QueryObjectType type, const QueryOptions& options);
std::set<std::string> resolveClockObjects(Database& database, const std::vector<std::string>& objects);
std::set<std::string> resolveExceptionObjects(Database& database, const std::vector<std::string>& objects);
std::vector<std::string> resolveClockSources(Database& database, const std::vector<std::string>& objects);
std::vector<std::string> resolveTypedObjects(Database& database, const std::vector<std::string>& objects, QueryObjectType type);

std::vector<std::string> resolveObjectList(Database& database, const std::vector<std::string>& object_list);
std::vector<std::string> resolveFullNames(Database& database, const std::string& object_list);
TimingPortConstraint& getPortConstraint(Database& database, const std::string& port_name);
void addQueryOptions(SdcTclCmd& command, bool hierarchical, bool exact, bool of_objects);
QueryOptions getQueryOptions(SdcTclCmd& command);

}  // namespace ista::sdc

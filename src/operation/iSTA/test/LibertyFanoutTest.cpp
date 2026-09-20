#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "Lib.hh"
#include "LibParserCpp.hh"

int main()
{
  const std::filesystem::path file = std::filesystem::temp_directory_path() / "ista_fanout_attributes.lib";
  {
    std::ofstream out(file);
    out << R"(library(fanout_test) {
      time_unit : "1ns";
      capacitive_load_unit(1,pf);
      default_fanout_load : 1.25;
      default_max_fanout : 12;
      cell(BUF) {
        pin(A) { direction : input; fanout_load : 2.5; }
        pin(Y) { direction : output; max_fanout : 3.5; }
      }
      cell(DEFAULTS) {
        pin(A) { direction : input; }
        pin(Y) { direction : output; }
      }
    })";
  }
  const std::string path = file.string();
  idb::LibertyReader reader(path.c_str());
  if (!reader.readLib() || !reader.linkLib()) {
    std::cerr << "Liberty parse failed\n";
    return 1;
  }
  std::unique_ptr<idb::LibBuilder> builder(reader.get_library_builder());
  idb::LibLibrary* lib = builder->get_lib();
  idb::LibCell* cell = lib->findCell("BUF");
  idb::LibCell* defaults = lib->findCell("DEFAULTS");
  if (!cell || !defaults || lib->get_default_fanout_load() != 1.25 || lib->get_default_max_fanout() != 12.0
      || cell->get_str2ports().at("A")->get_fanout_load() != 2.5 || cell->get_str2ports().at("Y")->get_max_fanout() != 3.5
      || defaults->get_str2ports().at("A")->get_fanout_load() || defaults->get_str2ports().at("Y")->get_max_fanout()) {
    std::cerr << "Liberty fanout attributes were lost or defaults overwrote explicit values\n";
    return 1;
  }
  std::cout << "Liberty fanout attributes: PASS\n";
}

#include <unistd.h>

#include "STAInterface.hpp"
#include "py_ista.h"

namespace python_interface {
bool initStaConfigMapByJSON(const std::string& config, std::map<std::string, std::any>& config_map);
void initStaConfigMapByDict(std::map<std::string, std::string>& config_dict, std::map<std::string, std::any>& config_map);
}  // namespace python_interface

int main()
{
  const std::filesystem::path file = std::filesystem::temp_directory_path() / ("ista_slew_config_" + std::to_string(getpid()) + ".json");
  try {
    for (const std::string value : {"0", "1", "2", "-1", "garbage", "1x", "0.5"}) {
      bool valid = value == "0" || value == "1";
      for (bool json : {false, true}) {
        std::map<std::string, std::any> result;
        bool accepted = true;
        try {
          if (json) {
            std::ofstream stream(file);
            stream << "{\"STA\":{\"-min_slew_degradation\":\"" << value << "\"}}";
            stream.close();
            if (!python_interface::initStaConfigMapByJSON(file.string(), result)) {
              throw std::runtime_error("JSON input missing");
            }
          } else {
            std::map<std::string, std::string> input = {{"-min_slew_degradation", value}};
            python_interface::initStaConfigMapByDict(input, result);
          }
        } catch (const std::invalid_argument&) {
          accepted = false;
        }
        if (accepted != valid || (accepted && std::any_cast<int32_t>(result.at("-min_slew_degradation")) != (value == "1"))) {
          throw std::runtime_error("Incorrect config conversion: " + value);
        }
      }
    }
    bool rejected = false;
    try {
      STAI.initSTA({{"-min_slew_degradation", int32_t(2)}});
    } catch (const std::invalid_argument&) {
      rejected = true;
    }
    if (!rejected) {
      throw std::runtime_error("C++ interface must reject invalid mode before initializing the design");
    }
    std::map<std::string, std::string> input;
    std::map<std::string, std::any> result;
    python_interface::initStaConfigMapByDict(input, result);
    if (result.contains("-min_slew_degradation")) {
      throw std::runtime_error("An absent option must preserve the C++ default");
    }
  } catch (const std::exception& error) {
    std::filesystem::remove(file);
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::filesystem::remove(file);
  ista::STAInterface::destroyInst();
  std::cout << "Python dictionary/JSON conversion and C++ boundary validation: PASS\n";
  return 0;
}


#include "dsopp/config_loader.hpp"

#include <any>
#include <filesystem>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "agent/agent.hpp"
#include "common/settings.hpp"
#include "dsopp/dsopp.hpp"

#include "energy/problems/photometric_bundle_adjustment/ceres_photometric_bundle_adjustment.hpp"
#include "feature_based_slam/tracker/fabric.hpp"
#include "marginalization/frame_marginalization_strategy.hpp"
#include "measures/similarity_measure_ssd.hpp"
#include "sanity_checker/fabric.hpp"
#include "sanity_checker/sanity_checker.hpp"

#include "agent/agent_settings.hpp"
#include "sensors/sensors.hpp"
#include "sensors/sensors_builder/camera_fabric.hpp"
#include "synchronizer/fabric.hpp"
#include "synchronizer/synchronizer.hpp"
#include "track/active_track.hpp"
#include "tracker/depth_estimators/depth_estimation.hpp"
#include "tracker/fabric.hpp"
#include "tracker/tracker.hpp"

namespace dsopp {

namespace {
/**
 * the list of required fields in config file
 *
 * If one of these fields is not in the parser, the agent will not be created
 */
const std::set<std::string> kRequiredFields = {"sensors", "time", "tracker", "gnss_alignmenter"};
/**
 * function to check the absence of required fields
 *
 * @param parser information from config file
 * @return true if one of the required fields is missing and false otherwise
 */
bool missingRequiredFields(YAML::Node &parser) {
  for (const auto &field : kRequiredFields) {
    if (!parser[field]) {
      LOG(WARNING) << "No " << field << " information in .yaml file";
      return true;
    }
  }
  return false;
}

std::string parseScalar(YAML::Node &scalar, bool canonize_path, const std::string &directory);
std::vector<std::any> parseSequence(YAML::Node &sequence, bool canonize_path, const std::string &directory);
std::map<std::string, std::any> parseMap(YAML::Node &map, bool canonize_path, const std::string &directory);
std::any parseNode(YAML::Node &node, bool canonize_path, const std::string &directory) {
  switch (node.Type()) {
    case YAML::NodeType::Undefined:
    case YAML::NodeType::Null:
      CHECK(true);
    case YAML::NodeType::Scalar:
      return parseScalar(node, canonize_path, directory);
    case YAML::NodeType::Sequence:
      return parseSequence(node, canonize_path, directory);
    case YAML::NodeType::Map:
      return parseMap(node, canonize_path, directory);
  }
  return std::any();
}
std::string parseScalar(YAML::Node &scalar, bool canonize_path, const std::string &directory) {
  if (canonize_path) {
    std::error_code error_code;
    auto absolute = std::filesystem::absolute(std::filesystem::path(directory) / scalar.as<std::string>());
    const auto canonized_path = std::filesystem::canonical(absolute, error_code).string();
    if (!error_code) {
      scalar = canonized_path;
    }
  }
  return scalar.as<std::string>();
}
std::vector<std::any> parseSequence(YAML::Node &sequence, bool canonize_path, const std::string &directory) {
  std::vector<std::any> elements;
  for (auto &element : sequence.as<std::vector<YAML::Node>>()) {
    elements.push_back(parseNode(element, canonize_path, directory));
  }
  return elements;
}
std::map<std::string, std::any> parseMap(YAML::Node &map, bool canonize_path, const std::string &directory) {
  std::map<std::string, std::any> parameters;
  for (auto &[key, value] : map.as<std::map<std::string, YAML::Node>>()) {
    parameters[key] = parseNode(value, canonize_path, directory);
  }
  return parameters;
}
std::map<std::string, std::any> parse(YAML::Node &parser, bool canonize_path = false,
                                      const std::string &directory = std::string()) {
  CHECK_EQ(parser.Type(), YAML::NodeType::Map);
  return parseMap(parser, canonize_path, directory);
}
/**
 * function to create sensors and to fill in the vector with sensors
 *
 * @param parser information from config file
 * @return Sensors object containing sensor information
 */
template <bool TRANSFORM_TO_PINHOLE>
void sensorBuilder(YAML::Node &parser, sensors::Sensors &sensors, sensors::AgentSettings &agent_settings) {
  for (auto sensor_config : parser["sensors"]) {
    auto parameters = parse(sensor_config);
    if (!parameters.contains("type")) {
      LOG(WARNING) << "Missing field \"type\" in the sensor parameters";
      continue;
    }
    const auto &sensor_type = std::any_cast<std::string>(parameters.at("type"));

    if (parameters.count("id") == 0) {
      LOG(WARNING) << "Missing field \"id\" in the sensor parameters";
      continue;
    }
    const auto &sensor_name = std::any_cast<std::string>(parameters.at("id"));
    if (sensor_type == "camera") {
      auto camera_transfromers = sensors::createCameraTransformer(parameters);
      auto camera_settings = sensors::createCameraSettings<TRANSFORM_TO_PINHOLE>(parameters, camera_transfromers);
      if (camera_settings) {
        auto &settings = agent_settings.addCameraSettings(std::move(*camera_settings));
        auto camera = sensors::createCamera(parameters, sensor_name, sensors.cameras().size(),
                                            std::move(camera_transfromers), settings);
        if (camera) {
          sensors.addCamera(std::move(camera));
        }
      }
    } else if (sensor_type == "gnss") {
      LOG(WARNING) << "contact Roadly INC for this functionality ";
    } else if (sensor_type == "imu") {
      LOG(WARNING) << "contact Roadly INC for this functionality ";
    } else {
      LOG(WARNING) << "Undefined sensor type";
      continue;
    }
  }
}

void modifyParameters(YAML::Node &config, const std::map<std::string, std::string> &config_args) {
  for (const auto &[key_string_ref, value] : config_args) {
    std::string key_string(key_string_ref);
    std::replace(key_string.begin(), key_string.end(), '.', ' ');
    std::istringstream iss(key_string);
    std::vector<std::string> keys(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());
    YAML::Node parent_node;
    parent_node.reset(config);

    for (const auto &key : keys) {
      if (parent_node.IsSequence()) {
        int idx = std::stoi(key);
        parent_node.reset(parent_node[idx]);
      } else {
        if (!parent_node[key]) {
          parent_node[key] = YAML::Node(YAML::NodeType::Map);
        }
        parent_node.reset(parent_node[key]);
      }
    }
    parent_node = value;
  }
}
}  // namespace
ConfigLoader::ConfigLoader(const std::string &file) : file_(file) {}

template <energy::motion::Motion Motion, bool TRANSFORM_TO_PINHOLE>
std::unique_ptr<DSOPP<Motion>> ConfigLoader::application(const std::map<std::string, std::string> &config_args) {
  YAML::Node parser = YAML::LoadFile(file_);

  modifyParameters(parser, config_args);

  if (missingRequiredFields(parser)) {
    return nullptr;
  }

  auto directory = std::filesystem::path(file_).parent_path();
  parse(parser, true, directory);

  auto agent = std::make_unique<Agent>();
  auto track = std::make_unique<track::ActiveTrack<Motion>>();
  sensorBuilder<TRANSFORM_TO_PINHOLE>(parser, agent->sensors(), track->agentSettings());

  const auto &time_parameters = parser["time"].as<std::map<std::string, std::string>>();
  auto synchronizer = synchronizer::create(time_parameters, agent->sensors());
  if (synchronizer == nullptr) {
    return nullptr;
  }

  auto tracker_config = parser["tracker"];
  const auto &init_parameters = parse(tracker_config);
  auto tracker =
      tracker::create<Motion, tracker::DepthEstimation, features::PixelMap>(init_parameters, agent->sensors());
  if (!tracker) return nullptr;

  auto initializer_config = parser["initializer"];
  const auto &monocular_initializer_parameters = parse(initializer_config);
  auto initializer = feature_based_slam::tracker::create(monocular_initializer_parameters, agent->sensors());
  if (!initializer) return nullptr;

  std::unique_ptr<sanity_checker::SanityChecker<track::ActiveTrack, Motion>> sanity_checker = nullptr;
  auto sanity_checker_config = parser["sanity_checker"];
  if (sanity_checker_config) {
    auto sanity_checker_parameters = parse(sanity_checker_config);
    sanity_checker = sanity_checker::create<track::ActiveTrack, Motion>(sanity_checker_parameters);
  }

  return std::make_unique<DSOPP<Motion>>(std::move(track), std::move(agent), std::move(synchronizer),
                                         std::move(tracker), std::move(initializer), std::move(sanity_checker));
}

template std::unique_ptr<DSOPP<energy::motion::SE3<Precision>>>
ConfigLoader::application<energy::motion::SE3<Precision>, true>(const std::map<std::string, std::string> &);
template std::unique_ptr<DSOPP<energy::motion::SE3<Precision>>>
ConfigLoader::application<energy::motion::SE3<Precision>, false>(const std::map<std::string, std::string> &);

}  // namespace dsopp

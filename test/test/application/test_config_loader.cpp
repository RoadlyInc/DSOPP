

#include "dsopp/config_loader.hpp"

#include "common/settings.hpp"
#include "dsopp/dsopp.hpp"
#include "energy/motion/motion.hpp"
#include "energy/motion/se3_motion.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <memory>

void rename_key(YAML::Node node, const std::string &key, const std::string &renamed_key) {
  for (auto it = node.begin(); it != node.end(); ++it)
    if (it->first.as<std::string>() == key) {
      it->first = renamed_key;
      return;
    }
}

namespace dsopp {

class testConfigLoader : public ::testing::Test {
 public:
  void SetUp() override {
    test_file_ = TEST_DATA_DIR "/track30seconds/config_loader_testing_config.yaml";

    reference_file_ = TEST_DATA_DIR "track30seconds/mono.yaml";
    general_node_ = YAML::LoadFile(reference_file_);
  }
  void TearDown() override { std::filesystem::remove_all(test_file_); }

  void write(const YAML::Node &node) {
    std::ofstream fout(test_file_);
    fout << node;
  }

  void checkNotNull(const YAML::Node &node) {
    write(node);
    ASSERT_NE(DSOPP<energy::motion::SE3<Precision>>::loadFromYaml(test_file_), nullptr);
  }

  void checkNull(const YAML::Node &node) {
    write(node);
    ASSERT_EQ(DSOPP<energy::motion::SE3<Precision>>::loadFromYaml(test_file_), nullptr);
  }

 protected:
  std::string test_file_;

  YAML::Node general_node_;
  std::string reference_file_;
};

TEST(testConfigLoaderMono, loadMono) {
  ASSERT_NE(DSOPP<energy::motion::SE3<Precision>>::loadFromYaml(MONO_CONFIG_FILE), nullptr);
}

TEST_F(testConfigLoader, validConfig) {
  YAML::Node node = YAML::Clone(general_node_);
  checkNotNull(node);
}

TEST_F(testConfigLoader, oneSensors) {
  YAML::Node node = YAML::Clone(general_node_);
  for (size_t i = node["sensors"].size() - 1; i > 0; --i) {
    node["sensors"].remove(i);
  }

  checkNotNull(node);
}

TEST_F(testConfigLoader, noRequiredGeneralField) {
  for (auto field : {"sensors", "time", "tracker"}) {
    YAML::Node node = YAML::Clone(general_node_);
    node.remove(field);
    checkNull(node);
  }
}

TEST_F(testConfigLoader, missingOrWrongSensorsField) {
  for (auto missing : {true, false}) {
    for (auto field : {"type", "provider", "id", "model"}) {
      YAML::Node node = YAML::Clone(general_node_);
      if (missing)
        node["sensors"][0].remove(field);
      else
        rename_key(node["sensors"][0], field, "wrongKey");
      checkNull(node);
    }
    // provider
    for (auto field : {"type", "video_file", "timestamps"}) {
      YAML::Node node = YAML::Clone(general_node_);
      if (missing)
        node["sensors"][0]["provider"].remove(field);
      else
        rename_key(node["sensors"][0]["provider"], field, "wrongKey");
      checkNull(node);
    }
    // model
    for (auto field : {"calibration"}) {
      YAML::Node node = YAML::Clone(general_node_);
      if (missing)
        node["sensors"][0]["model"].remove(field);
      else
        rename_key(node["sensors"][0]["model"], field, "wrongKey");
      checkNull(node);
    }
  }
}

TEST_F(testConfigLoader, wrongTimeTrackerValue) {
  for (auto field : {"type"}) {
    YAML::Node node = YAML::Clone(general_node_);
    node["time"][field] = "wrongValue";
    checkNull(node);
  }

  for (auto field : {"type", "sensor_id"}) {
    YAML::Node node = YAML::Clone(general_node_);
    node["tracker"][field] = "wrongValue";
    checkNull(node);
  }

  for (auto map : {"keyframe_strategy", "marginalization_strategy"}) {
    YAML::Node node = YAML::Clone(general_node_);
    node["tracker"][map]["strategy"] = "wrongValue";
    checkNull(node);
  }

  for (int i : {-10, -1, 0}) {
    YAML::Node node = YAML::Clone(general_node_);
    node["tracker"]["keyframe_strategy"]["factor"] = i;
    checkNull(node);
  }
}
}  // namespace dsopp

#ifndef MISSION_CONFIG_HPP
#define MISSION_CONFIG_HPP

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <unordered_map>

class MissionConfig {
public:

    MissionConfig(const std::string &config_file) {
        loadConfig(config_file);
    }

    bool loadConfig(const std::string &config_file) {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            if (!config["missions"]) {
                throw std::runtime_error("Missing 'missions' key in config file.");
            }

            for (const auto &mission : config["missions"]) {
                std::string name = mission.first.as<std::string>();

                mission_parameters_[name]["height_filter"] = mission.second["height_filter"].as<float>();
                mission_parameters_[name]["min_points"] = mission.second["min_points"].as<int>();
                mission_parameters_[name]["max_points"] = mission.second["max_points"].as<int>();
                mission_parameters_[name]["classification_distance_threshold"] = mission.second["classification_distance_threshold"].as<float>();
                mission_parameters_[name]["min_points_for_classification"] = mission.second["min_points_for_classification"].as<float>();
                mission_parameters_[name]["eps"] = mission.second["eps"].as<float>();
                mission_parameters_[name]["eps_reclustering"] = mission.second["eps_reclustering"].as<float>();
            }
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("MissionConfig"), "Failed to load config: %s", e.what());
            return false;
        }
    }

    float getFloatParam(const std::string &mission, const std::string &param, float default_value = 0.0) {
        if (mission_parameters_.count(mission) && mission_parameters_[mission].count(param)) {
            return std::get<float>(mission_parameters_[mission][param]);
        }
        return default_value;
    }

    int getIntParam(const std::string &mission, const std::string &param, int default_value = 0) {
        if (mission_parameters_.count(mission) && mission_parameters_[mission].count(param)) {
            return std::get<int>(mission_parameters_[mission][param]);
        }
        return default_value;
    }

private:
    std::unordered_map<std::string, std::unordered_map<std::string, std::variant<int, float>>> mission_parameters_;
};

#endif // MISSION_CONFIG_HPP

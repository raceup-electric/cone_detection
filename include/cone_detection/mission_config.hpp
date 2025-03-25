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

            // List of parameters and respective types
            const std::unordered_map<std::string, std::string> parameters = {
                {"min_points", "int"},
                {"max_points", "int"},
                {"min_points_for_classification", "int"},
                {"height_filter", "float"},
                {"classification_distance_threshold", "float"},
                {"eps", "float"},
                {"eps_reclustering", "float"},
                {"car_x_min", "float"},
                {"car_x_max", "float"},
                {"car_y_min", "float"},
                {"car_y_max", "float"},
                {"fov_x_min", "float"},
                {"fov_x_max", "float"},
                {"fov_y_min", "float"},
                {"fov_y_max", "float"}
            };

            // Load default parameters
            for (const auto& [parameter, type] : parameters) {
                if (!config[parameter]) {
                    throw std::runtime_error("Missing '" + parameter + "' key in config file.");
                }
                default_parameters_[parameter] = getParameterValue(config[parameter], type);
            }

            // Load mission-specific parameters
            for (const auto &mission : config["missions"]) {
                std::string name = mission.first.as<std::string>();
                for (const auto& [parameter, type] : parameters) {
                    if (mission.second[parameter]) {
                        mission_parameters_[name][parameter] = getParameterValue(mission.second[parameter], type);
                    }
                }
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
        if (default_parameters_.count(param)) {
            return std::get<float>(default_parameters_[param]);
        }
        return default_value;
    }

    int getIntParam(const std::string &mission, const std::string &param, int default_value = 0) {
        if (mission_parameters_.count(mission) && mission_parameters_[mission].count(param)) {
            return std::get<int>(mission_parameters_[mission][param]);
        }
        if (default_parameters_.count(param)) {
            return std::get<int>(default_parameters_[param]);
        }
        return default_value;
    }

private:
    std::unordered_map<std::string, std::unordered_map<std::string, std::variant<int, float>>> mission_parameters_;
    std::unordered_map<std::string, std::variant<int, float>> default_parameters_;

    std::variant<int, float> getParameterValue(const YAML::Node& node, const std::string& type) {
        if (type == "float") {
            return node.as<float>();
        } 
        if (type == "int") {
            return node.as<int>();
        }
        throw std::runtime_error("Invalid parameter type: " + type);
    }
};

#endif // MISSION_CONFIG_HPP

#ifndef MISSION_CONFIG_HPP
#define MISSION_CONFIG_HPP

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>

class MissionConfig {
public:
    int min_points;
    int max_points;
    int min_points_for_classification;
    float height_filter;
    float classification_distance_threshold;
    float eps;
    float eps_reclustering;
    float car_x_min;
    float car_x_max;
    float car_y_min;
    float car_y_max;
    float fov_x_min;
    float fov_x_max;
    float fov_y_min;
    float fov_y_max;

    MissionConfig() = default;
    MissionConfig(const std::string &config_file, const std::string &mission = "") {
        loadConfig(config_file, mission);
    }

    bool loadConfig(const std::string &config_file, const std::string &mission = "") {
        try {
            YAML::Node config = YAML::LoadFile(config_file);
            if (!config) {
                throw std::runtime_error("Invalid config file.");
            }

            loadParameter(config, mission, "min_points", min_points);
            loadParameter(config, mission, "max_points", max_points);
            loadParameter(config, mission, "min_points_for_classification", min_points_for_classification);
            loadParameter(config, mission, "height_filter", height_filter);
            loadParameter(config, mission, "classification_distance_threshold", classification_distance_threshold);
            loadParameter(config, mission, "eps", eps);
            loadParameter(config, mission, "eps_reclustering", eps_reclustering);
            loadParameter(config, mission, "car_x_min", car_x_min);
            loadParameter(config, mission, "car_x_max", car_x_max);
            loadParameter(config, mission, "car_y_min", car_y_min);
            loadParameter(config, mission, "car_y_max", car_y_max);
            loadParameter(config, mission, "fov_x_min", fov_x_min);
            loadParameter(config, mission, "fov_x_max", fov_x_max);
            loadParameter(config, mission, "fov_y_min", fov_y_min);
            loadParameter(config, mission, "fov_y_max", fov_y_max);

            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("MissionConfig"), "Failed to load config: %s", e.what());
            return false;
        }
    }

private:
    template <typename T>
    void loadParameter(const YAML::Node &config, const std::string &mission, const std::string &key, T &parameter) {
        if (!mission.empty() && config["missions"] && config["missions"][mission] && config["missions"][mission][key]) {
            parameter = config["missions"][mission][key].as<T>();
        } else if (config[key]) {
            parameter = config[key].as<T>();
        } else {
            throw std::runtime_error("Missing '" + key + "' key in config file.");
        }
    }
};

#endif // MISSION_CONFIG_HPP

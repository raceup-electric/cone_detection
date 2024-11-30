/*
g++ -std=c++11 -o plot_algo plot_algo.cpp -I$HOME/matplotlib-cpp -I/usr/include/python3.8 -lpython3.8 -lboost_system -lboost_filesystem
*/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <matplotlibcpp.h>  // You need to include the matplotlibcpp header for plotting

namespace plt = matplotlibcpp;

int main() {
    std::ifstream file("algo_time.txt");

    if (!file) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    std::vector<double> time_diff;   // Declare the vector to store time differences

    std::string line;
    while (std::getline(file, line)) {
        std::cout << "Read line: " << line << std::endl;  // Debug print

        // Check if the line contains the keyword "Time:"
        if (line.find("Time:") != std::string::npos) {
            std::string received_str = line.substr(line.find(":") + 1);
            time_diff.push_back(std::stod(received_str));  // Store the time difference
        }
    }

    file.close();  // Close the file after reading

    // Plotting the time flow
    plt::plot(time_diff);  // Plot the vector containing time differences
    plt::xlabel("Index");
    plt::ylabel("Time (seconds)");
    plt::title("Time Flow Over Iterations");

    // Show the plot
    plt::show();

    return 0;
}

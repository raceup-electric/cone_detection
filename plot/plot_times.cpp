#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <matplotlibcpp.h>

/*
g++ -std=c++11 -o plot_times plot_times.cpp -I$HOME/matplotlib-cpp -I/usr/include/python3.8 -lpython3.8 -lboost_system -lboost_filesystem
*/

namespace plt = matplotlibcpp;

int main() {
    std::ifstream file("timestamp.txt");

    if (!file) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    std::vector<double> received;   // Declare received vector
    std::vector<double> published;  // Declare published vector

    std::string line;
    while (std::getline(file, line)) {
        std::cout << "Read line: " << line << std::endl;  // Debug print

        if (line.find("Received:") != std::string::npos) {
            std::string received_str = line.substr(line.find(":") + 1);
            received.push_back(std::stod(received_str));
        }
        if (line.find("Published:") != std::string::npos) {
            std::string published_str = line.substr(line.find(":") + 1);
            published.push_back(std::stod(published_str));
        }
    }

    std::cout << "Received data count: " << received.size() << std::endl;
    std::cout << "Published data count: " << published.size() << std::endl;

    if (received.size() == 0 || published.size() == 0) {
        std::cerr << "No data to plot." << std::endl;
        return 1;
    }

    // Plotting
    plt::plot(received, published);
    plt::xlabel("Received (s)");
    plt::ylabel("Published (s)");
    plt::show();

    return 0;
}

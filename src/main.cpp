#include "hybrid.hpp"

#include <boost/program_options.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char **argv) {
    namespace po = boost::program_options;
    po::options_description description("Standalone NRF-SAT + earliest-failure Min-RC flow");
    description.add_options()
        ("help,h", "Show options")
        ("map,m", po::value<std::string>()->required(), "Native or MovingAI map")
        ("scenario,s", po::value<std::string>()->required(), "Native A U or legacy U A tasks scenario")
        ("time,t", po::value<double>()->default_value(30), "Time limit in seconds")
        ("max-makespan", po::value<int>()->default_value(-1), "Optional horizon cap; -1 means no artificial cap")
        ("output,o", po::value<std::string>()->required(), "Append results to this CSV")
        ("solution", po::value<std::string>(), "Write validated paths as role,agent,time,x,y CSV")
        ("trace", po::value<std::string>(), "Write learned cut clauses as CSV");
    try {
        po::variables_map args;
        po::store(po::parse_command_line(argc, argv, description), args);
        if (args.count("help")) { std::cout << description << '\n'; return 0; }
        po::notify(args);
        // Reject accidental overlap between inputs and generated artifacts.
        std::vector<std::filesystem::path> used;
        for (const auto *name : {"map", "scenario", "output", "solution", "trace"}) {
            if (!args.count(name)) continue;
            const auto path = std::filesystem::weakly_canonical(args[name].as<std::string>());
            for (const auto &other : used) {
                if (path == other) throw std::invalid_argument("Input/output paths must all be different");
            }
            used.push_back(path);
        }
        const auto problem = hybrid::read_problem(args["map"].as<std::string>(), args["scenario"].as<std::string>());
        hybrid::Options options;
        options.time_limit = args["time"].as<double>();
        options.max_makespan = args["max-makespan"].as<int>();
        std::ofstream trace;
        if (args.count("trace")) {
            const auto path = std::filesystem::path(args["trace"].as<std::string>());
            if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
            trace.open(path);
            if (!trace) throw std::runtime_error("Cannot open trace output");
            options.trace = &trace;
        }
        const auto result = hybrid::solve(problem, options);
        if (result.status == "solved" && args.count("solution")) {
            hybrid::write_solution_csv(args["solution"].as<std::string>(), problem, result);
        }
        hybrid::write_result_csv(args["output"].as<std::string>(), problem, result);
        std::cout << hybrid::algorithm_name << ": " << result.status << " " << std::fixed << std::setprecision(3)
                  << result.runtime_seconds << "s";
        if (result.status == "solved") std::cout << " makespan=" << result.makespan;
        std::cout << " flow_failures=" << result.flow_failures << " cut_clauses=" << result.cut_clauses << '\n';
        if (!result.reason.empty()) std::cout << result.reason << '\n';
        return result.status == "solved" ? 0 : (result.status == "timeout" ? 124 : 2);
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}

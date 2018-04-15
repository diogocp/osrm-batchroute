#include <osrm/coordinate.hpp>
#include <osrm/engine_config.hpp>
#include <osrm/json_container.hpp>
#include <osrm/osrm.hpp>
#include <osrm/route_parameters.hpp>
#include <osrm/status.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>


void parse_program_options(const int argc, const char *argv[],
                           osrm::EngineConfig &config,
                           std::string &input_path, std::string &output_path) {
    namespace fs = boost::filesystem;
    namespace po = boost::program_options;

    fs::path base_path;
    std::string algorithm;

    // Generic options
    po::options_description generic_options("Options");
    generic_options.add_options()
            ("help,h", "Show this help message")
            ("input,i", po::value<std::string>(&input_path), "input file (defaults to stdin)")
            ("output,o", po::value<std::string>(&output_path), "output file (defaults to stdout)");

    // Configuration options
    po::options_description config_options("Configuration");
    config_options.add_options()
            ("algorithm,a",
             po::value<std::string>(&algorithm)->default_value("CH"),
             "Algorithm to use. Can be CH or MLD.");

    // Hidden options
    po::options_description hidden_options("Hidden options");
    hidden_options.add_options()
            ("base,b", po::value<fs::path>(&base_path)->required(), "base path to .osrm file");

    // Positional options
    po::positional_options_description positional_options;
    positional_options.add("base", 1);

    // Combine above options for parsing
    po::options_description cmdline_options;
    cmdline_options.add(generic_options).add(config_options).add(hidden_options);

    std::string usage;
    if (argc > 0) {
        const std::string executable = fs::path(argv[0]).filename().string();
        usage = "Usage: " + executable + " <base.osrm> [<options>]";
    }
    po::options_description visible_options(usage);
    visible_options.add(generic_options).add(config_options);

    // Parse command line options
    po::variables_map option_variables;
    try {
        po::store(po::command_line_parser(argc, argv)
                          .options(cmdline_options)
                          .positional(positional_options)
                          .run(),
                  option_variables);
    } catch (const po::error &e) {
        std::cerr << e.what() << std::endl;
        std::cerr << visible_options;
        std::exit(2);
    }

    if (option_variables.count("help")) {
        std::cerr << visible_options;
        std::exit(EXIT_SUCCESS);
    }

    try {
        po::notify(option_variables);
    } catch (const po::required_option &e) {
        std::cerr << "Error: "
                  << cmdline_options.find(e.get_option_name().substr(2), false).description()
                  << " must be specified" << std::endl;
        std::cerr << visible_options;
        std::exit(2);
    }

    if (algorithm == "CH" || algorithm == "ch") {
        config.algorithm = osrm::EngineConfig::Algorithm::CH;
    } else if (algorithm == "MLD" || algorithm == "mld") {
        config.algorithm = osrm::EngineConfig::Algorithm::MLD;
    } else {
        std::cerr << "Error: invalid algorithm " << algorithm << std::endl;
        std::exit(-1);
    }

    config.use_shared_memory = false;
    config.storage_config = osrm::storage::StorageConfig(base_path);
}


int calc_route(const osrm::OSRM &osrm,
               const osrm::util::Coordinate &orig,
               const osrm::util::Coordinate &dest,
               double &distance,
               double &duration) {
    using namespace osrm;

    osrm::RouteParameters params;
    params.alternatives = 0;
    params.coordinates.push_back(orig);
    params.coordinates.push_back(dest);

    osrm::json::Object result;
    const auto status = osrm.Route(params, result);

    if (status == Status::Ok) {
        auto &routes = result.values["routes"].get<json::Array>();
        auto &route = routes.values.at(0).get<json::Object>();

        distance = route.values["distance"].get<json::Number>().value;
        duration = route.values["duration"].get<json::Number>().value;

        return 0;
    } else if (status == Status::Error) {
        const auto code = result.values["code"].get<json::String>().value;
        const auto message = result.values["message"].get<json::String>().value;

        std::cerr << "Error: " << code << ". " << message << std::endl;
    }

    return -1;
}


int main(int argc, const char *argv[]) {
    osrm::EngineConfig config;
    std::string input_path, output_path;

    parse_program_options(argc, argv, config, input_path, output_path);
    const osrm::OSRM osrm{config};


    // Set up the input and output streams
    std::streambuf *inbuf, *outbuf;
    std::ifstream infile;
    std::ofstream outfile;

    if (input_path.empty()) {
        // Read input from cin
        inbuf = std::cin.rdbuf();
        std::cin.tie(nullptr);
    } else {
        // Read input from file
        infile.open(input_path);
        if (!infile.is_open()) {
            std::perror("Failed to open input file");
            return EXIT_FAILURE;
        }
        inbuf = infile.rdbuf();
    }
    if (output_path.empty()) {
        // Write output to cout
        outbuf = std::cout.rdbuf();
        std::cout.tie(nullptr);
    } else {
        // Write output to file
        outfile.open(output_path);
        if (!outfile.is_open()) {
            std::perror("Failed to open output file");
            return EXIT_FAILURE;
        }
        outbuf = outfile.rdbuf();
    }
    std::istream in(inbuf);
    std::ostream out(outbuf);


    // Main loop
    std::string line;
    while (std::getline(in, line)) {
        // Remove carriage return from the end of the line, if present
        if (line.size() && line[line.size() - 1] == '\r') {
            line.pop_back();
        }

        std::vector<std::string> split_line;
        boost::split(split_line, line, [](char c) { return c == ','; });

        if (split_line.size() != 4) {
            std::cerr << "Malformed line: " << line << '\n';
            return EXIT_FAILURE;
        }

        errno = 0;
        const double lat1 = std::atof(split_line[0].c_str());
        const double lng1 = std::atof(split_line[1].c_str());
        const double lat2 = std::atof(split_line[2].c_str());
        const double lng2 = std::atof(split_line[3].c_str());
        if (errno) {
            std::perror("Error while parsing coordinates");
            return EXIT_FAILURE;
        }

        double distance, duration;
        const int result = calc_route(osrm,
                                      {osrm::util::FloatLongitude{lng1}, osrm::util::FloatLatitude{lat1}},
                                      {osrm::util::FloatLongitude{lng2}, osrm::util::FloatLatitude{lat2}},
                                      distance, duration);
        if (result == 0) {
            out << line << ',' << distance << ',' << duration << '\n';
        }
    }
    if (in.bad()) {
        perror("Error while reading file");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

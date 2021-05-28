#include <iostream>
#include <cstdio>
#include <string>
#include <algorithm>
#include "tinyxml2/tinyxml2.h"
#include "argparse/argparse.hpp"
#include "configio.h"
#include "algo.h"
#include <omp.h>

std::string strToLower(const std::string& str) {
    std::string str2 = str;
    std::for_each(str2.begin(), str2.end(), [](char& c){
        c = static_cast<char>(std::tolower(c));
    });
    return str2;
}

int main(int argc, char* argv[]) {
    argparse::ArgumentParser program("delaytool");

    program.add_argument("input")
            .help("input xml file with network resources and virtual links");

    program.add_argument("output")
            .help("output xml file with delays of all VLs to all their destinations");

    program.add_argument("-s", "--scheme")
            .help("scheme schemeName: voqa|voqb|oqp|oqa|oqb (default: oqp). oqp ~ OQ; oqa, oqb ~ CIOQ")
            .default_value(std::string("OqPacket"))
            .action([](const std::string& value) {
                static const std::map<std::string, std::string> mapping = {
                        {"voqa", "VoqA"},
                        {"voqb", "VoqB"},
                        {"oqp", "OqPacket"},
                        {"oqa", "OqA"},
                        {"oqb", "OqB"},
                        {"oqc", "OqCellB"}, // DEBUG
                        {"mock", "Mock"}, // DEBUG
                };
                auto found = mapping.find(strToLower(value));
                if(found != mapping.end()) {
                    return found->second;
                } else {
                    throw std::runtime_error("invalid value of -s");
                }
            });

    program.add_argument("-c", "--cellsize")
            .help("cell size in bytes for cell scheme (VOQ-A/B, OQ-A/B)")
            .action([](const std::string& value) { return std::stoi(value); })
            .default_value(cellSizeDefault);

    program.add_argument("-p", "--periodvoq")
            .help("clock period for VOQ scheme")
            .action([](const std::string& value) { return std::stoi(value); })
            .default_value(voqPeriodDefault);

    program.add_argument("--printconfig")
            .implicit_value(true)
            .default_value(false)
            .help("print verbose info about resources and VL configuration");

    program.add_argument("--printdelays")
            .implicit_value(true)
            .default_value(false)
            .help("print calculated E2E delays");

    program.add_argument("-j", "--jitdef")
            .action([](const std::string& value) { return std::stof(value); })
            .default_value(static_cast<float>(jitStartDefault))
            .help("default start jitter in microseconds if not specified in input data (default: 500)");

    program.add_argument("-r", "--rate")
            .action([](const std::string& value) { return std::stoi(value); })
            .default_value(0)
            .help("change (force) link rate to specified value, in byte/ms");

    program.add_argument("-f", "--factor")
            .action([](const std::string& value) { return std::stof(value); })
            .default_value(1.f)
            .help("multiply max packet size for all VLs by factor (and cast to integer)");

    program.add_argument("--bpmaxit")
            .action([](const std::string& value) { return static_cast<uint64_t>(std::stof(value)); })
            .default_value(static_cast<uint64_t>(100000))
            .help(std::string("max number of iterations of calculating busy period for oqp/oqa/oqb.\n")
                  + "its calculation won't be endless because its parameters are checked for a sign of this earlier,"
                  + "but it may take too long. set 0 for no restrictions.");

    program.add_argument("-t", "--threads")
            .help("number of threads (OpenMP)")
            .action([](const std::string& value) { return std::stoi(value); })
            .default_value(0);

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cout << program;
        return 0;
    }

    std::string fileIn = program.get<std::string>("input");
    std::string fileOut = program.get<std::string>("output");
    std::string scheme = program.get<std::string>("--scheme");
    int cellSize = program.get<int>("--cellsize");
    int voqPeriod = program.get<int>("--periodvoq");
    float startJitDefault = program.get<float>("--jitdef");
    int forceLinkRate = program.get<int>("--rate");
    float sizeFactor = program.get<float>("--factor");
    bool printConfig = program.get<bool>("--printconfig");
    bool printDelays = program.get<bool>("--printdelays");
    int num_threads = program.get<int>("--threads");
    omp_set_num_threads(num_threads > 0 ? num_threads : 1);
    uint64_t bpMaxIter = program.get<uint64_t>("--bpmaxit");

    tinyxml2::XMLDocument doc;
    auto err = doc.LoadFile(fileIn.c_str());
    if(err) {
        fprintf(stderr, "error: can't load input file: %s\n", tinyxml2::XMLDocument::ErrorIDToName(err));
        return 0;
    }
    FILE *fpOut = fopen(fileOut.c_str(), "w");
    if(fpOut == nullptr) {
        fprintf(stderr, "error: can't open output file: %s\n", fileOut.c_str());
        return 0;
    }
    VlinkConfigOwn config = fromXml(doc, scheme, cellSize, voqPeriod,
            startJitDefault, forceLinkRate, sizeFactor, bpMaxIter);
    if(config == nullptr) {
        fprintf(stderr, "error reading from xml\n");
        fclose(fpOut);
        return 0;
    }
    if(printConfig) {
        DebugInfo(config.get());
    }
    auto bwUsage = config->bwUsage();
    if(!bwCorrect(bwUsage)) {
        fprintf(stderr, "error: bandwidth usage is more than 100%%\n");
        fclose(fpOut);
        return 0;
    }
    auto bwStats = getStats(bwUsage);
    printf("bwUsage: min=%f, max=%f, mean=%f, var=%f\n",
           bwStats.min, bwStats.max, bwStats.mean, bwStats.var);
    if(config->scheme == "OqA" || config->scheme == "OqB") {
        auto bwUsageCells = config->bwUsage(true);
        if(!bwCorrect(bwUsageCells)) {
            fprintf(stderr, "error: bandwidth usage in cells is more than 100%%\n");
            fclose(fpOut);
            return 0;
        }
//        auto bwStatsCells = getStats(bwUsageCells);
//        printf("cells bwUsage: min=%f, max=%f, mean=%f, var=%f\n",
//               bwStatsCells.min, bwStatsCells.max, bwStatsCells.mean, bwStatsCells.var);
    }
    try {
        Error calcErr = config->detectCycles(false);
        if(calcErr) {
            fprintf(stderr, "error calculating delay because of invalid VL configuration: %s, %s\n",
                    calcErr.TypeString().c_str(), calcErr.Verbose().c_str());
            fclose(fpOut);
            return 0;
        }
        if(num_threads > 0) {
            calcErr = config->calcE2eParallel(printDelays);
        } else {
            calcErr = config->calcE2e(printDelays);
        }
        if(calcErr) {
            fprintf(stderr, "error calculating delay because of invalid VL configuration: %s, %s\n",
                    calcErr.TypeString().c_str(), calcErr.Verbose().c_str());
            fclose(fpOut);
            return 0;
        }
        printf("calcs=%d\n", config->calcs); // DEBUG
        printf("_________OK________\n"); // DEBUG
    } catch(std::exception& e) {
        fprintf(stderr, "error calculating delay because of exception: %s\n", e.what());
    }
    bool ok = toXml(config.get(), doc);
    if(!ok) {
        fprintf(stderr, "error converting to xml\n");
        fclose(fpOut);
        return 0;
    }
    err = doc.SaveFile(fpOut, false);
    if(err) {
        fprintf(stderr, "error writing to output file: %s\n", tinyxml2::XMLDocument::ErrorIDToName(err));
    }
    fclose(fpOut);
    return 0;
}

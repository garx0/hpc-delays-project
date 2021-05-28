#include <set>
#include "tinyxml2/tinyxml2.h"
#include "argparse/argparse.hpp"
#include "configio.h"
#include "algo.h"

void deletePaths(tinyxml2::XMLDocument& doc, int vlId, std::set<int> dests) {
    auto afdxxml = doc.FirstChildElement("afdxxml");
    auto vls = afdxxml->FirstChildElement("virtualLinks");
    tinyxml2::XMLElement* curVl = nullptr;
    for(auto vl = vls->FirstChildElement("virtualLink");
        vl != nullptr;
        vl = vl->NextSiblingElement("virtualLink"))
    {
        if (std::stoi(vl->Attribute("number")) == vlId) {
            curVl = vl;
            break;
        }
    }
    std::vector<tinyxml2::XMLElement*> pathsToDelete;
    for(auto path = curVl->FirstChildElement("path");
        path != nullptr;
        path = path->NextSiblingElement("path"))
    {
        if(dests.find(std::stoi(path->Attribute("dest"))) != dests.end()) {
            pathsToDelete.push_back(path);
        }
    }
    for(auto el: pathsToDelete) {
        curVl->DeleteChild(el);
    }
    if(curVl->FirstChildElement("path") == nullptr) {
        vls->DeleteChild(curVl);
    }
}

int main(int argc, char* argv[]) {
    argparse::ArgumentParser program("deletepaths");

    program.add_argument("input")
            .help("input xml file with network resources and virtual links");

    program.add_argument("output")
            .help("output xml file with some VL paths deleted and without cycles");

    program.add_argument("-r", "--random")
            .implicit_value(true)
            .default_value(false)
            .help("randomness in graph traversal order");

    program.add_argument("-s", "--seed")
            .help("seed for randomness in graph traversal order")
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
    bool shuffle = program.get<bool>("--random");
    int seed = program.get<int>("--seed");

    printf("test\n");
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
    srand(seed);
    printf("test\n");
    while(true) {
        VlinkConfigOwn config = fromXml(doc, "Mock", cellSizeDefault, voqPeriodDefault);
        if(config == nullptr) {
            fprintf(stderr, "error reading from xml\n");
            fclose(fpOut);
            return 0;
        }
        Error cycleErr = config->detectCycles(shuffle);
        if(cycleErr != Error::Cycle) {
            assert(cycleErr == Error::Success);
            break;
        }
        assert(!cycleErr.VerboseRaw().empty());
        printf("%s\n", cycleErr.Verbose().c_str());
        std::vector<int> deleteArgs = TokenizeCsv(cycleErr.VerboseRaw());
        std::set<int> pathArgs;
        for(auto it = deleteArgs.begin() + 1; it != deleteArgs.end(); ++it) {
            pathArgs.insert(*it);
        }
        deletePaths(doc, deleteArgs[0], pathArgs);
    }
    err = doc.SaveFile(fpOut, false);
    if(err) {
        fprintf(stderr, "error writing to output file: %s\n", tinyxml2::XMLDocument::ErrorIDToName(err));
    }
    fclose(fpOut);
    return 0;
}
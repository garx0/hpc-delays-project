#include <iostream>
#include <cstdio>
#include <vector>
#include <map>
#include <string>
#include <cassert>
#include <cmath>
#include <sstream>
#include <memory>
#include "configio.h"

std::vector<int> TokenizeCsv(const std::string& str) {
    std::vector<int> res;
    std::stringstream ss(str);
    int num;
    while(ss >> num) {
        res.push_back(num);
        if (ss.peek() == ',' || ss.peek() == ' ') {
            ss.ignore();
        }
    }
    return res;
}

// doc can be modified. it will be used when exporting config to xml with new data
VlinkConfigOwn fromXml(tinyxml2::XMLDocument& doc, const std::string& scheme,
        int cellSize, int voqPeriod, double jitDefaultValue, int forceLinkRate,
        double loadFactor, uint64_t bpMaxIter)
{
    VlinkConfigOwn config = std::make_unique<VlinkConfig>();
    config->calcs = 0; // DEBUG
    try {
        auto afdxxml = doc.FirstChildElement("afdxxml");
        auto resources = afdxxml->FirstChildElement("resources");
        config->linkRate = forceLinkRate == 0
                           ? static_cast<int64_t>(
                                   std::stof(resources->FirstChildElement("link")->Attribute("capacity")))
                           : forceLinkRate;
        config->cellSize = cellSize;
        config->voqL = voqPeriod;
        config->scheme = scheme;
        config->bpMaxIter = bpMaxIter;

        for(auto res = resources->FirstChildElement("link");
             res != nullptr;
             res = res->NextSiblingElement("link")) {
            if(forceLinkRate == 0 && std::stof(res->Attribute("capacity")) != config->linkRate) {
                std::cerr << "error: bad input resources - all links must have the same capacity" << std::endl;
                return nullptr;
            } else if(forceLinkRate != 0) {
                res->SetAttribute("capacity", forceLinkRate);
            }
            int port1 = std::stoi(res->Attribute("from"));
            int port2 = std::stoi(res->Attribute("to"));
            config->links[port1] = port2;
            config->links[port2] = port1;
        }

        // device id -> vector of IDs of its ports
        std::map<int, std::vector<int>> portNums;

        for(auto res = resources->FirstChildElement("endSystem");
             res != nullptr;
             res = res->NextSiblingElement("endSystem")) {
            std::vector<int> ports = TokenizeCsv(res->Attribute("ports"));
            if(ports.size() != 1) {
                std::cerr << "error: bad input - end systems must have one port" << std::endl;
                return nullptr;
            }
            int number = std::stoi(res->Attribute("number"));
            config->_portDevice[ports[0]] = number;
            config->devices[number] = std::make_unique<Device>(config.get(), Device::End, number);
            portNums[number] = ports;
        }

        for(auto res = resources->FirstChildElement("switch");
             res != nullptr;
             res = res->NextSiblingElement("switch")) {
            int number = std::stoi(res->Attribute("number"));
            std::vector<int> ports = TokenizeCsv(res->Attribute("ports"));
            for(auto portId: ports) {
                config->_portDevice[portId] = number;
            }
            config->devices[number] = std::make_unique<Device>(config.get(), Device::Switch, number);
            portNums[number] = ports;
        }
        // add ports
        for(auto[num, ports] : portNums) {
            config->getDevice(num)->AddPorts(ports);
        }

        auto vls = afdxxml->FirstChildElement("virtualLinks");
        for(auto vl = vls->FirstChildElement("virtualLink");
             vl != nullptr;
             vl = vl->NextSiblingElement("virtualLink")) {
            std::vector<std::vector<int>> paths;
            int number = std::stoi(vl->Attribute("number"));
            int srcId = std::stoi(vl->Attribute("source"));
            int bag = std::stoi(vl->Attribute("bag"));
            int smax = std::stoi(vl->Attribute("lmax"));
            if(loadFactor != 1.0) {
                smax = static_cast<int>(smax * loadFactor);
                vl->SetAttribute("lmax", smax);
            }
            int smin = std::min(sminDefault, smax);
            vl->SetAttribute("lmin", smin);
            auto jitStr = vl->Attribute("jitStart"); // in us
            double jit0 = (jitStr ? std::stof(jitStr) : jitDefaultValue) / 1e3; // in ms
            for(auto pathEl = vl->FirstChildElement("path");
                 pathEl != nullptr;
                 pathEl = pathEl->NextSiblingElement("path")) {
                std::vector<int> path = TokenizeCsv(pathEl->Attribute("path"));
                assert(!path.empty());
                paths.push_back(path);
            }
            config->vlinks[number] = std::make_unique<Vlink>(config.get(), number, srcId, paths, bag, smax, smin, jit0);
        }
        assert(!config->vlinks.empty());
    } catch(std::exception& e) {
        fprintf(stderr, "exception while reading vl config: %s\n", e.what());
        return nullptr;
    };
    printf("%ld vlinks\n", config->vlinks.size());
    if(config->scheme == "VoqA" || config->scheme == "VoqB") {
        printf("VOQ period = %.1f ms\n", config->linkByte2ms(config->voqL * config->cellSize));
    }
    return config;
}

// adding maxDelay and maxJit attributes to VL paths with max e2e delay and jitter values in us
// and scheme attributes for each switch
// doc must already contain the resources and VL configuration
// (e.g. doc used for building config)
bool toXml(VlinkConfig* config, tinyxml2::XMLDocument& doc) {
    auto afdxxml = doc.FirstChildElement("afdxxml");
    auto resources = afdxxml->FirstChildElement("resources");
    for(auto res = resources->FirstChildElement("switch");
        res != nullptr;
        res = res->NextSiblingElement("switch"))
    {
        res->SetAttribute("scheme", config->scheme.c_str());
        if(config->scheme != "OqPacket") {
            res->SetAttribute("cellSize", config->cellSize);
        }
        if(config->scheme == "VoqA" || config->scheme == "VoqB") {
            res->SetAttribute("processingPeriod", config->voqL);
        }
    }
    auto vls = afdxxml->FirstChildElement("virtualLinks");
    for(auto vlEl = vls->FirstChildElement("virtualLink");
        vlEl != nullptr;
        vlEl = vlEl->NextSiblingElement("virtualLink"))
    {
        int number = std::stoi(vlEl->Attribute("number"));
        Vlink* vl = config->getVlink(number);
        for(auto path = vlEl->FirstChildElement("path");
            path != nullptr;
            path = path->NextSiblingElement("path"))
        {
            int deviceId = std::stoi(path->Attribute("dest"));
            auto found = vl->dst.find(deviceId);
            assert(found != vl->dst.end());
            Vnode* vnode = found->second;
            path->SetAttribute("maxDelay",
                    static_cast<int>(ceil(1000. * config->linkByte2ms(vnode->e2e.dmax()))));
            path->SetAttribute("maxJit",
                    static_cast<int>(ceil(1000. * config->linkByte2ms(vnode->e2e.jit()))));
        }
    }
    return true;
}

stats_t getStats(std::map<int, double> data) {
    assert(!data.empty());
    double min = data.begin()->second;
    double max = min;
    double sum = 0;
    double sum2 = 0;
    for(auto [_, val]: data) {
        if(val < min) {
            min = val;
        }
        if(val > max) {
            max = val;
        }
        sum += val;
        sum2 += val * val;
    }
    size_t n = data.size();
    double mean = sum / n;
    double var = sqrt((sum2 - sum * sum / n) / (n - 1));
    return {min, max, mean, var};
}

bool bwCorrect(std::map<int, double> bwUsage) {
    for(auto [_, val]: bwUsage) {
        if(val > 1.) {
            return false;
        }
    }
    return true;
}

void VlinkPathDebugInfo(const Vnode* dest, const std::string& prefix) {
    if(dest->prev != nullptr) {
        VlinkPathDebugInfo(dest->prev, prefix);
        printf("%sdevice %d, prev output port %d, input port %d, after %d\n",
               prefix.c_str(), dest->device->id, dest->outPrev, dest->in->id, dest->prev->device->id);
    } else {
        printf("%sdevice %d, is source\n",
               prefix.c_str(), dest->device->id);
    }
}

void DebugInfo(const VlinkConfig* config) {
    for(auto vl: config->getAllVlinks()) {
        printf("vl %d:\n", vl->id);
        for(const auto [_, dst]: vl->dst) {
            printf("vl %d path to dst %d:\n", dst->vl->id, dst->device->id);
            VlinkPathDebugInfo(dst, "\t");
            printf("\n");
        }
        printf("\n");
    }
    printf("\n");
    for(auto device: config->getAllDevices()) {
        printf("device %d:\n", device->id);
        for(auto port: device->getAllPorts()) {
            printf("\tinput port %d:", port->id);
            auto inVnodes = port->getAllVnodes();
            printf(" %ld vls:", inVnodes.size());
            if(!inVnodes.empty()) {
                printf(" vl");
                for(auto vnode: inVnodes) {
                    printf(" %d", vnode->vl->id);
                }
            }
            printf("\n");
            printf("\toutput port %d:", port->id);
            auto outVnodes = device->fromOutPort(port->id)->getAllVnodes();
            printf(" %ld vls:", outVnodes.size());
            if(!outVnodes.empty()) {
                printf(" vl");
                for(auto vnode: outVnodes) {
                    printf(" %d", vnode->vl->id);
                }
            }
            printf("\n");
        }
    }
    printf("\n");
}
#include <iostream>
#include <cstdio>
#include <vector>
#include <cmath>
#include <random>
#include "algo.h"
#include "delay.h"

std::vector<int> idxRange(int size, bool shuffle) {
    std::vector<int> res;
    res.reserve(size);
    for(int i = 0; i < size; i++) {
        res.push_back(i);
    }
    if(shuffle) {
        for(int i = size - 1; i > 0; i--) {
            std::swap(res[i], res[rand() % i]);
        }
    }
    return res;
}

bool operator==(Error::ErrorType lhs, const Error& rhs) {
    return lhs == rhs.type;
}

bool operator!=(Error::ErrorType lhs, const Error& rhs) {
    return lhs != rhs.type;
}

void Device::AddPorts(const std::vector<int>& portIds) {
    for(int i: portIds) {
        // we only have port number
        ports[i] = std::make_unique<Port>(this, i);
    }
}

Port* Device::fromOutPort(int outPortId) const {
    auto inPortId = config->connectedPort(outPortId);
    auto deviceId = config->portDevice(inPortId);
    auto device = config->getDevice(deviceId);
    return device->getPort(inPortId);
}

Port* Device::getPort(int portId) const {
    auto found = ports.find(portId);
    assert(found != ports.end());
    return found->second.get();
}

std::vector<Port*> Device::getAllPorts() const {
    std::vector<Port*> res;
    res.reserve(ports.size());
    for(const auto& pair: ports) {
        res.push_back(pair.second.get());
    }
    return res;
}

Port::Port(Device* device, int id)
    : id(id), device(device)
{
    // calculate outPrev, prevDevice by number and links and portDevice in device->config
    VlinkConfig* config = device->config;
    outPrev = config->connectedPort(id);
    int prevDeviceId = config->portDevice(outPrev);
    prevDevice = config->getDevice(prevDeviceId);
    delays = config->factory->Create(config->scheme, this);
}

Vnode* Port::getVnode(int vlId) const {
    auto found = vnodes.find(vlId);
    assert(found != vnodes.end());
    return found->second;
}

std::vector<Vnode*> Port::getAllVnodes() const {
    std::vector<Vnode*> res;
    res.reserve(vnodes.size());
    for(const auto& pair: vnodes) {
        res.push_back(pair.second);
    }
    return res;
}

double Port::bwUsage(bool cells) const {
    double sum = 0;
    for(auto vnode: getAllVnodes()) {
        int s = vnode->vl->smax;
        int c = device->config->cellSize;
        s += cells * (c * (s % c != 0) - s % c); // sizeRound formula
        sum += static_cast<double>(s) / vnode->vl->bagB;
    }
    return sum;
}

Vlink::Vlink(VlinkConfig* config, int id, int srcId, std::vector<std::vector<int>> paths,
    int bag, int smax, int smin, double jit0)
    : config(config), id(id), bag(bag), bagB(bag * config->linkRate),
    smax(smax), smin(smin), jit0(jit0), jit0b(std::ceil(jit0 * config->linkRate))
{
    assert(!paths.empty());
    src = std::make_unique<Vnode>(this, srcId);
    src->device->sourceFor.push_back(this);

    for(const auto& path: paths) {
        int i = -1;
        Vnode* vnodeNext = src.get();
        Vnode* vnode = vnodeNext; // unnecessary if src != nullptr
        while(vnodeNext != nullptr) {
//            assert(i < path.size() - 1);
            vnode = vnodeNext;
            vnodeNext = vnode->selectNext(path[++i]);
        }
        // vnode is initialized if make_unique didnt returned nullptr
        // now we need to add a new-made Vnode of input port path[i] to vnode->next vector
        for(size_t j = i; j < path.size(); j++) {
            vnode->next.push_back(std::make_unique<Vnode>(this, path[j], vnode));
            vnode = vnode->next[vnode->next.size()-1].get();
        }
        // vnode is a leaf
        dst[vnode->device->id] = vnode;
    }
}

Vnode::Vnode(Vlink* vlink, int portId, Vnode* prev)
    : config(vlink->config), vl(vlink),
      device(vlink->config->getDevice(vlink->config->portDevice(portId))),
      prev(prev),
      in(prev != nullptr ? device->getPort(portId) : nullptr),
      outPrev(in != nullptr ? in->outPrev : -1),
      cycleState(NotVisited), calcPrepared(false), calcQueued(false), e2e()
{
    in->vnodes[vl->id] = this;
}

// for source end system
Vnode::Vnode(Vlink* vlink, int srcId)
    : config(vlink->config), vl(vlink),
      device(vlink->config->getDevice(srcId)),
      prev(nullptr), in(nullptr), outPrev(-1),
      cycleState(NotVisited), calcPrepared(false), calcQueued(false), e2e()
{}

Error Vnode::prepareTest(bool shuffle) {
    if(cycleState == NotVisited) {
        cycleState = VisitedNotPrepared;
        config->calcs += 1;
    } else if(cycleState == VisitedNotPrepared) {
        std::string verbose =
                std::string("cycle: entered VL ")
                + std::to_string(vl->id)
                + " device "
                + std::to_string(device->id)
                + " again. suggestion: delete VL "
                + std::to_string(vl->id)
                + " paths to end systems:";
        std::string verboseRaw = std::to_string(vl->id);
        for(auto dest: getAllDests()) {
            verbose += " " + std::to_string(dest->device->id);
            verboseRaw += " " + std::to_string(dest->device->id);
        }
        return Error(Error::Cycle, verbose, verboseRaw);
    }
    assert(prev != nullptr);
    if(prev->prev != nullptr) {
        Port* fromOutPort = prev->device->fromOutPort(outPrev);
        auto vnodeNextList = fromOutPort->getAllVnodes();
        auto order = idxRange(vnodeNextList.size(), shuffle);
        for(auto idx: order) {
            auto curVnode = vnodeNextList[idx]->prev;
            if(curVnode->cycleState != Prepared) {
                Error err = curVnode->prepareTest(shuffle);
                if(err) {
                    return err;
                }
            }
        }
    }
    cycleState = Prepared;
    return Error::Success;
}

Error Vnode::prepareCalc() {
    assert(prev != nullptr);
    Error err;
    DelayData delay;
    if(prev->prev == nullptr) {
        std::tie(delay, err) = in->delays->calc(vl, true);
        if(err) {
            return err;
        }
        in->delays->setDelay(delay);
    } else {
        Port* fromOutPort = prev->device->fromOutPort(outPrev);
        for(auto vnodeNext: fromOutPort->getAllVnodes()) {
            auto curVnode = vnodeNext->prev;
            if(!curVnode->calcPrepared) {
                err = curVnode->prepareCalc();
                if(err) {
                    return err;
                }
            }
            if(!in->delays->readyIn()) { // fix for orig
                int vlId = curVnode->vl->id;
//                printf("LINE %d: getting delay from map\n", __LINE__); // DEBUG
                auto delay = curVnode->in->delays->getDelay(vlId);
                assert(delay.ready());
                in->delays->setInDelay(delay);
            }
        }
        in->delays->setInputReady();
        std::tie(delay, err) = in->delays->calc(vl);
        if(err) {
            return err;
        }
        in->delays->setDelay(delay);
    }
    calcPrepared = true;
    return Error::Success;
}

Task::Task(Port* _port, Vlink* _vl, bool _first): port(_port), vl(_vl), first(_first) {
    vnode = _port->getVnode(_vl->id);
}

Error Task::calc() {
    DelayData delay;
    Error err;
    std::tie(delay, err) = port->delays->calc(vl, first);
    if(err) {
        return err;
    }
    result = delay;
    return Error::Success;
}

// queues all tasks that already have input data but weren't calculated or queued already
Error Vnode::queueCalc(std::vector<Task>& tasks) {
    assert(prev != nullptr);
    Error err;
    if(calcQueued) {
        return Error::Success;
    }
    if(prev->prev == nullptr) {
        tasks.emplace_back(in, vl, true);
//        printf("queued VL %d to input %d of device %d (from ES)\n", vl->id, in->id, in->device->id); // DEBUG
        calcQueued = true;
    } else {
        Port* fromOutPort = prev->device->fromOutPort(outPrev);
        if(!in->delays->readyIn()) {
            bool readyIn = true;
            for(auto vnodeNext: fromOutPort->getAllVnodes()) {
                auto curVnode = vnodeNext->prev;
                readyIn = readyIn && curVnode->calcPrepared;
                if(curVnode->calcPrepared) {
                    int vlId = curVnode->vl->id;
//                    printf("LINE %d: getting delay from map\n", __LINE__); // DEBUG
                    auto delay = curVnode->in->delays->getDelay(vlId);
                    assert(delay.ready());
                    in->delays->setInDelay(delay);
                } else if(!curVnode->calcQueued) {
                    err = curVnode->queueCalc(tasks);
                    if(err) {
                        return err;
                    }
                }
            }
            if(readyIn) {
                // input data for calculating this delay is ready
                in->delays->setInputReady();
            }
        }
        if(in->delays->readyIn()) {
            // input data for calculating this delay is ready
            tasks.emplace_back(in, vl, false);
//            printf("queued VL %d to input %d of device %d\n", vl->id, in->id, in->device->id); // DEBUG
            calcQueued = true;
        }
    }
    return Error::Success;
}

Error Vnode::calcE2e() {
    Error err = prepareCalc();
    if(!err) {
        e2e = in->delays->e2e(vl->id);
    }
    return err;
}

Error Vnode::queueE2e(std::vector<Task>& tasks) {
    if(calcPrepared) {
        e2e = in->delays->e2e(vl->id);
        return Error::Success;
    } else if(!calcQueued) {
        Error err = queueCalc(tasks);
        return err;
    } else {
        return Error::Success;
    }
}

Vnode* Vnode::selectNext(int portId) const {
    for(const auto &own: next) {
        if(own->in->id == portId) {
            return own.get();
        }
    }
    return nullptr;
}

std::vector<const Vnode*> Vnode::getAllDests() const {
    std::vector<const Vnode*> vec;
    _getAllDests(vec);
    return vec;
}

void Vnode::_getAllDests(std::vector<const Vnode*> &vec) const {
    if(next.empty()) {
        assert(device->type == Device::End);
        vec.push_back(this);
        return;
    }
    for(const auto& nxt: next) {
        nxt->_getAllDests(vec);
    }
}

Error VlinkConfig::calcE2e(bool print) {
    for(auto vl: getAllVlinks()) {
        for(auto [_, vnode]: vl->dst) {
            Error err = vnode->calcE2e();
            if(err) {
                return err;
            }
            if(print) {
                DelayData e2e = vnode->e2e;
                printf("VL %d to %d: maxDelay = %li lB (%.0f us), jit = %li lB (%.0f us), minDelay = %li lB (%.0f us)\n",
                       vnode->vl->id, vnode->device->id,
                       e2e.dmax(), linkByte2ms(e2e.dmax()) * 1e3,
                       e2e.jit(), linkByte2ms(e2e.jit()) * 1e3,
                       e2e.dmax() - e2e.jit(), linkByte2ms(e2e.dmax() - e2e.jit()) * 1e3);
            }
        }
    }
    if(scheme == "voqa" || scheme == "voqb") {
        for(auto device: getAllDevices()) {
            // check sums by switch input ports
            if(device->type != Device::Switch) {
                continue;
            }
            Error err = completeCheckVoq(device);
            if(err) {
                return err;
            }
        }
    }
    return Error::Success;
}

Error VlinkConfig::calcE2eParallel(bool print) {
    std::vector<Task> tasks;
    tasks.reserve(calcs);
    while(true) {
        printf("---------------------------------------------1. start adding tasks\n"); // DEBUG
        tasks.clear();
        for(auto vl: getAllVlinks()) {
            for(auto[_, vnode]: vl->dst) {
                Error err = vnode->queueE2e(tasks);
                if(err) {
                    return err;
                }
            }
        }
        printf("----------------------------------------------2. added %ld tasks\n", tasks.size()); // DEBUG
        if(tasks.empty()) {
            break;
        }
#pragma omp parallel default(none) shared(tasks)
        {
            int tid = omp_get_thread_num();
#pragma omp for schedule(dynamic, 20)
            for(size_t i = 0; i < tasks.size(); i++) {
//                printf("[%d] start task %ld\n", tid, i); // DEBUG
                Task& task = tasks[i];
                Error err = task.calc();
                if(err) {
                    task.result = DelayData();
                    printf("[%d] ERROR: %s\n", tid, err.Verbose().c_str());
                }
//                printf("[%d] end task %ld: VL %d input port %d (device %d)\n", tid, i,
//                       task.vl->id, task.port->id, task.port->device->id); // DEBUG
            }
#pragma omp barrier
        }
        printf("-----------------------------------------------3. start setting results\n"); // DEBUG
        for(size_t i = 0; i < tasks.size(); i++) {
            Task& task = tasks[i];
            task.port->delays->setDelay(task.result);
            task.vnode->calcPrepared = true;
        }
        printf("------------------------------------------------4. end   setting results\n"); // DEBUG
    }

    if(print) {
        for(auto vl: getAllVlinks()) {
            for(auto [_, vnode]: vl->dst) {
                DelayData e2e = vnode->e2e;
                printf("VL %d to %d: maxDelay = %li lB (%.0f us), jit = %li lB (%.0f us), minDelay = %li lB (%.0f us)\n",
                       vnode->vl->id, vnode->device->id,
                       e2e.dmax(), linkByte2ms(e2e.dmax()) * 1e3,
                       e2e.jit(), linkByte2ms(e2e.jit()) * 1e3,
                       e2e.dmax() - e2e.jit(), linkByte2ms(e2e.dmax() - e2e.jit()) * 1e3);
            }
        }
    }

    if(scheme == "voqa" || scheme == "voqb") {
        for(auto device: getAllDevices()) {
            // check sums by switch input ports
            if(device->type != Device::Switch) {
                continue;
            }
            Error err = completeCheckVoq(device);
            if(err) {
                return err;
            }
        }
    }
    return Error::Success;
}

Error VlinkConfig::detectCycles(bool shuffle) {
    std::vector<Vnode*> dests;
    for(auto vl: getAllVlinks()) {
        for(auto [_, vnode]: vl->dst) {
            dests.push_back(vnode);
        }
    }
    auto order = idxRange(dests.size(), shuffle);
    for(auto idx: order) {
        auto vnode = dests[idx];
        Error err = vnode->prepareTest(shuffle);
        if(err) {
            return err;
        }
    }
    return Error::Success;
}

Vlink* VlinkConfig::getVlink(int id) const {
    auto found = vlinks.find(id);
    assert(found != vlinks.end());
    return found->second.get();
}

Device* VlinkConfig::getDevice(int id) const {
    auto found = devices.find(id);
    assert(found != devices.end());
    return found->second.get();
}

int VlinkConfig::connectedPort(int portId) const {
    auto found = links.find(portId);
    assert(found != links.end());
    return found->second;
}

int VlinkConfig::portDevice(int portId) const {
    auto found = _portDevice.find(portId);
    assert(found != _portDevice.end());
    return found->second;
}

std::vector<Vlink*> VlinkConfig::getAllVlinks() const {
    std::vector<Vlink*> res;
    res.reserve(vlinks.size());
    for(const auto& pair: vlinks) {
        res.push_back(pair.second.get());
    }
    return res;
}

std::vector<Device*> VlinkConfig::getAllDevices() const {
    std::vector<Device*> res;
    res.reserve(devices.size());
    for(const auto& pair: devices) {
        res.push_back(pair.second.get());
    }
    return res;
}

VlinkConfig::VlinkConfig(): factory(std::make_unique<PortDelaysFactory>()) {}

std::map<int, double> VlinkConfig::bwUsage(bool cells) {
    std::map<int, double> res;
    for(auto device: getAllDevices()) {
        for(auto port: device->getAllPorts()) {
            res[port->id] = port->bwUsage(cells);
        }
    }
    return res;
}

void PortDelays::setInDelays(const std::map<int, DelayData> &values) {
    inDelays = values;
    _inputReady = true;
}

void PortDelays::setInDelay(DelayData delay) {
    inDelays[delay.vl()->id] = delay;
}

void PortDelays::setDelay(DelayData delay) {
    delays[delay.vl()->id] = delay;
}

int64_t PortDelays::trMax(Vlink* vl) const {
    switch(cellType) {
        case P: return vl->smax;
        case A: return vl->smax + vl->config->cellSize * (vl->smax % vl->config->cellSize != 0)
                       - vl->smax % vl->config->cellSize + vl->config->cellSize; // sizeRound(smax, cellSize) + cellSize
        case B: return vl->config->cellSize * 2;
        default: return 0;
    }
}

int64_t PortDelays::trMin(Vlink* vl) const {
    switch(cellType) {
        case P: return vl->smin;
        case A: return vl->smin;
        case B: return std::min(vl->smin, vl->config->cellSize);;
        default: return 0;
    }
}

std::pair<DelayData, Error> PortDelays::calc(Vlink* vl, bool first) {
//    if(delays[vl->id].ready()) {
//        printf("calc called again=====================\n"); // DEBUG
//        DelayData delaydata = delays[vl->id];
//        return std::make_pair(delaydata, Error(Error::Success));
//    }
//#pragma omp critical // DEBUG
//    {
//        vl->config->calcs += 1; // DEBUG
//    }
    if(!first) {
        return calcCommon(vl);
    } else {
        return std::make_pair(DelayData(vl, trMin(vl), vl->jit0b + trMax(vl) - trMin(vl)), Error::Success);
    }
}

DelayData PortDelays::e2e(int vlId) const {
//    printf("LINE %d: getting delay from map\n", __LINE__); // DEBUG
    auto delay = getDelay(vlId);
    auto vl = delay.vl();
    assert(delay.ready());
    int64_t dmin = delay.dmin() - trMin(vl) + vl->smin;
    int64_t dmax = delay.dmax() - trMax(vl) + vl->smax;
    assert(dmin >= 0);
    assert(dmax >= 0);
    return DelayData(vl, dmin, dmax - dmin);
}

DelayData PortDelays::getFromMap(int vl, const std::map<int, DelayData> &delaysMap) {
    auto found = delaysMap.find(vl);
    assert(found != delaysMap.end());
    return found->second;
}

// register Creators for all PortDelays types
void PortDelaysFactory::RegisterAll() {
    AddCreator<Mock>("Mock");
    AddCreator<VoqA>("VoqA");
    AddCreator<VoqB>("VoqB");
    AddCreator<OqPacket<>>("OqPacket");
    AddCreator<OqA>("OqA");
    AddCreator<OqB>("OqB");
    AddCreator<OqCellB>("OqCellB"); // DEBUG
}

PortDelaysOwn PortDelaysFactory::Create(const std::string& name, Port* port) {
    auto found = creators.find(name);
    if(found != creators.end()) {
        return found->second->Create(port);
    } else {
        throw std::logic_error("invalid PortDelays schemeName");
    }
}

int defaultIntMap::Get(int key) {
    auto found = find(key);
    return found == end() ? 0 : found->second;
}

void defaultIntMap::Inc(int key, int val) {
    auto found = find(key);
    if(found == end()) {
        (*this)[key] = val;
    } else {
        (*this)[key] += val;
    }
}

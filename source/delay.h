#pragma once
#ifndef DELAYTOOL_DELAY_H
#define DELAYTOOL_DELAY_H
#include "algo.h"

// floor(x/y)
inline int64_t floordiv(int64_t x, int64_t y) {
    return x / y;
}

// ceil(x/y), x >= 0
inline int64_t ceildiv(int64_t x, int64_t y) {
    return x / y + (x % y != 0);
}

// = ceil(x/y + 0) = floor(x/y) + 1, x >= 0
inline int64_t ceildiv_up(int64_t x, int64_t y) {
    return x / y + 1;
}

// round size of packet to whole number of cells
inline int64_t sizeRound(int64_t size, int64_t cellSize) {
    return size + cellSize * (size % cellSize != 0) - size % cellSize;
}

// round size of packet to whole number of cells if round == true
inline int64_t sizeRound(int64_t size, int64_t cellSize, bool round) {
    return size + round * (cellSize * (size % cellSize != 0) - size % cellSize);
}

inline int64_t numPackets(int64_t intvl, int64_t bag, int64_t jit) {
    return ceildiv(intvl + jit, bag);
}

// = numPackets(intvl+0, ...) (limit from above)
inline int64_t numPacketsUp(int64_t intvl, int64_t bag, int64_t jit) {
    return ceildiv_up(intvl + jit, bag);
}

// intvl >= 0
inline int64_t numCells(int64_t intvl, int64_t bag, int64_t jit, int64_t nCells, int64_t cellSize) {
    return nCells * ((intvl + jit) / bag)
           + std::min(nCells,
                      ceildiv((intvl + jit) % bag - jit * ((intvl + jit) / bag == 0), cellSize));
}

// = numCells(intvl+0, ...) (limit from above)
// intvl >= 0 (interval)
inline int64_t numCellsUp(int64_t intvl, int64_t bag, int64_t jit, int64_t nCells, int64_t cellSize) {
    return nCells * ((intvl + jit) / bag)
           + std::min(nCells,
                      ceildiv_up((intvl + jit) % bag - jit * ((intvl + jit) / bag == 0), cellSize));
}

inline std::string VoqOverloadVerbose(int portId, int switchId, const std::string& portType) {
    return std::string("overload on ")
           + portType
           + " port "
           + std::to_string(portId)
           + " of switch "
           + std::to_string(switchId);
}

inline std::string OqOverloadVerbose(Port* port) {
    return std::string("overload on output port ")
           + std::to_string(port->outPrev)
           + " of switch "
           + std::to_string(port->prevDevice->id)
           + " because busy period calculation took over "
           + std::to_string(port->device->config->bpMaxIter)
           + " iterations";
}

int64_t busyPeriod(const std::map<int, DelayData>& inDelays, VlinkConfig* config, bool byTick = false);

class Mock : public PortDelays
{
public:
    Mock(Port* port): PortDelays(port, "Mock", P) {}

    std::pair<DelayData, Error> calcCommon(Vlink* vl) override;
};

class defaultIntMap : public std::map<int, int> {
public:
    defaultIntMap() = default;

    int Get(int key);

    void Inc(int key, int val);
};

// base class for VoqA and VoqB
class Voq : public PortDelays
{
public:
    Voq(Port* port, const std::string& schemeName, celltype_t cellType)
        : PortDelays(port, schemeName, cellType), outPrevLoadSum(0), loadReady(false) {}

    // C_l values by l on prev switch
    defaultIntMap vlinkLoad;

    // C_pj values by p on prev switch, where j ~ this->port->outPrev
    defaultIntMap outPrevLoad;

    // sum of outPrevLoad values
    int outPrevLoadSum;

    // is information about input load (outPrevLoad and its sum) ready
    bool loadReady;

    void calcOutPrevLoad();

    virtual void calcVlinkLoad() = 0;
};

class VoqA : public Voq
{
public:
    explicit VoqA(Port* port): Voq(port, "VoqA", A) {}

    void calcVlinkLoad() override;

    std::pair<DelayData, Error> calcCommon(Vlink* vl) override;
};

class VoqB : public Voq
{
public:
    explicit VoqB(Port* port): Voq(port, "VoqB", B) { }

    void calcVlinkLoad() override;

    std::pair<DelayData, Error> calcCommon(Vlink* vl) override;
};

// if cells = true, skmax is rounded up to whole number of cells
template<bool cells = false>
class OqPacket : public PortDelays
{
public:
    explicit OqPacket(Port* port)
        : PortDelays(port, "OqPacket<" + std::to_string(cells) + ">", P), bp(-1) {}

    // == Rk,j(t) - Jk, k == vl->id
    int64_t delayFunc(int64_t t, Vlink* vl) const;

    // == Rk,j(q)* - Jk, k == vl->id
    int64_t delayFuncRem(int q, Vlink* vl);

    std::pair<DelayData, Error> calcCommon(Vlink* vl) override;

private:
    int64_t bp;
};

// OqB without packet FIFO
class OqCellB : public PortDelays
{
public:
    explicit OqCellB(Port* port)
        : PortDelays(port, "OqCellB", B), bp(-1) {}

    // == Rk,j(t) - Jk, k == vl->id
    int64_t delayFunc(int64_t t, Vlink* vl) const;

    // == Rk,j(q)* - Jk, k == vl->id
    int64_t delayFuncRem(int q, Vlink* vl);

    std::pair<DelayData, Error> calcCommon(Vlink* vl) override;

private:
    int64_t bp;
};

// two different schemes stitched together.
// although, works correctly only if Scheme1 out delay is calculated
// as if next (not current) switch has cell type P
template<class Scheme1, class Scheme2, PortDelays::celltype_t ct>
class TwoSchemes : public PortDelays
{
    static_assert(std::is_base_of<PortDelays, Scheme1>::value && std::is_base_of<PortDelays, Scheme2>::value,
                  "parameters of TwoSchemes must inherit from PortDelays");
public:
    explicit TwoSchemes(Port* port)
        : PortDelays(port, "TwoSchemes", ct),
        midDelaysReady(false), scheme1(port), scheme2(port)
    {
        schemeName += "<" + scheme1.schemeName + ", " + scheme2.schemeName + ">";
    }

    std::pair<DelayData, Error> calcCommon(Vlink* vl) override;

private:
    bool midDelaysReady;
    Scheme1 scheme1;
    Scheme2 scheme2;
};

using OqA = TwoSchemes<OqPacket<true>, OqPacket<>, PortDelays::A>;
using OqB = TwoSchemes<OqCellB, OqPacket<>, PortDelays::B>;

// == Rk,j(t) - Jk, k == curVlId
template<bool cells>
int64_t OqPacket<cells>::delayFunc(int64_t t, Vlink* curVl) const {
    int64_t res = -t;
    for(auto const [vlId, delay]: inDelays) {
        auto vl = delay.vl();
        res += numPacketsUp(t, vl->bagB, (vlId != curVl->id) * delay.jit()) * vl->smax;
    }
    return res;
}

// == Rk,j(q)* - Jk, k == curVlId
template<bool cells>
int64_t OqPacket<cells>::delayFuncRem(int q, Vlink* curVl) {
    int64_t value = 0;
    int64_t bags = (q - 1) * curVl->bagB;
    for(auto const [vlId, delay]: inDelays) {
        auto vl = delay.vl();
        value += vl->smax * (
                vl->id != curVl->id
                ? numPacketsUp(std::min(bp - curVl->smax, bags), vl->bagB, delay.jit())
                : q);
    }
    return std::min(bp, value) - bags;
}

template<bool cells>
std::pair<DelayData, Error> OqPacket<cells>::calcCommon(Vlink* curVl) {
    if(bp < 0) {
        bp = busyPeriod(inDelays, config, cells);
        if(bp < 0) {
            return std::make_pair(
                    DelayData(),
                    Error(Error::BpTooLong, OqOverloadVerbose(port)));
        }
    }
    int64_t delayFuncMax = -1;
    int64_t delayFuncValue;

    // calc delayFunc in chosen points, part 1
    for(int64_t t = 0; t <= bp - curVl->smax; t += curVl->bagB) {
        delayFuncValue = delayFunc(t, curVl);
        if(delayFuncValue > delayFuncMax) {
            delayFuncMax = delayFuncValue;
        }
    }

    // calc delayFunc in chosen points, part 2
    DelayData curDelay;
    for(auto const [vlId, delay]: inDelays) {
        if(vlId == curVl->id) {
            curDelay = delay;
            continue;
        }
        auto vl = delay.vl();
        for(int64_t t = sizeRound(delay.jit(), vl->bagB) - delay.jit();
            t <= bp - curVl->smax;
            t += vl->bagB)
        {
            delayFuncValue = delayFunc(t, curVl);
            if(delayFuncValue > delayFuncMax) {
                delayFuncMax = delayFuncValue;
            }
        }
    }

    // calc delayFuncRem in chosen points
    int qMin = numPacketsUp(bp - curVl->smin, curVl->bagB, 0);
    int qMax = numPackets(bp, curVl->bagB, curDelay.jit());
    for(int q = qMin; q <= qMax; q++) {
        delayFuncValue = delayFuncRem(q, curVl);
        if(delayFuncValue > delayFuncMax) {
            delayFuncMax = delayFuncValue;
        }
    }
    assert(delayFuncMax >= 0);
    int64_t dmax = delayFuncMax + curDelay.dmax();
    int64_t dmin = curDelay.dmin() + sizeRound(curVl->smin, config->cellSize, cells);
    assert(dmax >= dmin);
    return std::make_pair(
            DelayData(curVl, dmin, dmax-dmin),
            Error(Error::Success));
}

template<class Scheme1, class Scheme2, PortDelays::celltype_t ct>
std::pair<DelayData, Error> TwoSchemes<Scheme1, Scheme2, ct>::calcCommon(Vlink* curVl) {
    DelayData delay;
    Error err;
    if(!midDelaysReady) {
        scheme1.setInDelays(inDelays);
        for(auto [vlId, delay]: inDelays) {
            std::tie(delay, err) = scheme1.calc(delay.vl());
            if(err) {
                return std::make_pair(DelayData(), err);
            }
            scheme1.setDelay(delay);
        }
        scheme2.setInDelays(scheme1.getDelays());
        midDelaysReady = true;
    }
    std::tie(delay, err) = scheme2.calc(curVl);
    if(err) {
        return std::make_pair(DelayData(), err);
    }
//    scheme2.setDelay(delay);
    int64_t dmax = delay.dmax() + trMax(curVl) - scheme2.trMax(curVl);
    int64_t dmin = delay.dmin() + trMin(curVl) - scheme2.trMin(curVl);
    assert(dmin <= dmax);

    return std::make_pair(
            DelayData(curVl, dmin, dmax - dmin),
            Error(Error::Success));
}

#endif //DELAYTOOL_DELAY_H

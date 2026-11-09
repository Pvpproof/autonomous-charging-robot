#pragma once
#include <cstdint>
#include <string>
#include <ctime>
inline uint64_t source_monotonic_ns() {
    timespec ts{};
    if (clock_gettime(CLOCK_MONOTONIC,&ts)!=0) return 0;
    return uint64_t(ts.tv_sec)*1000000000ULL+uint64_t(ts.tv_nsec);
}
// Updated only on fully received controller packets, never on ROS publication.
struct SourceFeedbackState {
    uint64_t tick=0, progress_ns=0;
    bool valid=false, latched=false;
    std::string fault="no controller frame";
    bool accept(uint16_t len, uint64_t magic, uint64_t timer, uint64_t now_ns) {
        if (latched) return false;
        if (len!=1440 || magic!=0x0123456789ABCDEFULL || timer==0 || now_ns==0) {
            valid=false; fault="invalid controller frame"; return false;
        }
        if (tick && timer<tick) {
            valid=false; latched=true; fault="controller timestamp regressed"; return false;
        }
        if (timer==tick) return false; // Preserve age of last actual source progress.
        tick=timer; progress_ns=now_ns; valid=true; fault.clear(); return true;
    }
};

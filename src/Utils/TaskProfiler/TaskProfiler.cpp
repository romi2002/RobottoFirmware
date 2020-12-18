//
// Created by abiel on 12/17/20.
//

#include "TaskProfiler.h"
#include "Utils/Math/ExponentialMovingAverage.h"
#include <vector>
#include <algorithm>

std::map<String, TaskProfilerData>::iterator TaskProfiler::initProfiler(const String &taskData) {
    TaskProfilerData profilerData;
    auto ret = timingData.emplace(taskData, profilerData);
    return ret.first;
}

void TaskProfiler::updateProfiler(const std::map<String, TaskProfilerData>::iterator &it) {
    TaskProfilerData &data(it->second);
    double ms = (data.timeSinceRun) * 1e-3;
    data.avg = exponentialMovingAverage(data.avg, ms);
    data.timeSinceRun = 0;
}

void TaskProfiler::outputData(Stream *stream) {
    std::vector<std::pair<double, String>> outData;

    for(const auto &data : timingData){
        const TaskProfilerData &profData(data.second);

        outData.emplace_back(0,String());
        auto it = outData.end() - 1;

        it->first = profData.avg;
        it->second = "Task: "; it->second += data.first;
        it->second += "   Average: "; it->second += profData.avg;
        it->second += '\n';
    }

    std::sort(outData.begin(), outData.end(), [](const std::pair<double, String> &a, const std::pair<double, String> &b){
        return a.first > b.first;
    });

    for(const auto &data : outData){
        stream->print(data.second);
    }
}
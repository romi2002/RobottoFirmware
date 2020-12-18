//
// Created by abiel on 12/17/20.
//

#ifndef TEENSYROSCONTROLLER_TASKPROFILER_H
#define TEENSYROSCONTROLLER_TASKPROFILER_H

#include <Arduino.h>
#include <map>

struct TaskProfilerData{
    elapsedMicros timeSinceRun;
    double avg{0};
};

typedef  std::map<String, TaskProfilerData>::iterator TaskProfilerIt;

class TaskProfiler {
public:
    TaskProfilerIt initProfiler(const String &taskData);

    static void updateProfiler(const TaskProfilerIt &it);

    void outputData(Stream *stream);

    static TaskProfiler& getInstance(){
        static TaskProfiler instance;
        return instance;
    }

private:
    TaskProfiler() = default;

    std::map<String, TaskProfilerData> timingData;
};


#endif //TEENSYROSCONTROLLER_TASKPROFILER_H

#include "ns3/core-module.h"
#include "ns3/data-processor.h"
#include "json.hpp"

#include <chrono>
#include <iostream>
#include <random>

using namespace ns3;
using json = nlohmann::json;

// Random number generator for a and b
const unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::mt19937 gen(seed);
std::uniform_int_distribution<int> distrib(1, 10);
// Data processor (south bound)
auto dataProcessor = CreateObject<DataProcessor>();

Time measStartTime;
Time measInterval;
int actionWaitTimeMs;
Time stopTime;

void
GenerateMeasurement()
{
    auto a = distrib(gen);
    auto b = distrib(gen);
    std::cout << "at " << Simulator::Now().ToDouble(Time::MS) << " ms, " << "measurement: a=" << a << ", b=" << b << std::endl;

    // Create one measurement that contains a and b
    auto meas = CreateObject<NetworkStats>("calculator", 0, Simulator::Now().GetMilliSeconds());
    meas->Append("addend::a", a);
    meas->Append("addend::b", b);
    dataProcessor->AppendMeasurement(meas);
    Simulator::Schedule(measInterval, &GenerateMeasurement);
}

void
RecvAction(const json& action)
{
    if (action == nullptr)
    {
        return;
    }
    std::cout << "at " << Simulator::Now().ToDouble(Time::MS) << " ms, " << "action: sum=" << action.get<int>() << std::endl;
}

int
main(int argc, char* argv[])
{
    // Parse env config
    std::ifstream jsonStream("env-configure.json");
    json jsonConfig;
    try
    {
        jsonStream >> jsonConfig;
    }
    catch (const json::type_error& e)
    {
        std::cout << "message: " << e.what() << '\n'
                  << "exception id: " << e.id << std::endl;
    }
    measStartTime = MilliSeconds(jsonConfig["measurement_start_time_ms"].get<int>());
    measInterval = MilliSeconds(jsonConfig["measurement_interval_ms"].get<int>());
    actionWaitTimeMs = jsonConfig["max_wait_time_for_action_ms"].get<int>();
    stopTime = MilliSeconds(jsonConfig["env_end_time_ms"].get<int>());

    Simulator::Schedule(measStartTime, &DataProcessor::StartMeasurement, dataProcessor);
    Simulator::Schedule(measStartTime, &GenerateMeasurement);
    dataProcessor->SetMaxPollTime(actionWaitTimeMs);    // timeout for zmq_poll
    dataProcessor->SetNetworkGymActionCallback("calculator::sum", 0, MakeCallback(&RecvAction));

    Simulator::Stop(stopTime);
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}

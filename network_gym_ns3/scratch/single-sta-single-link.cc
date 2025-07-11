/*
 * Copyright (c) 2025 University of Washington
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 */

// This is a program to evaluate link performance in Wi-Fi 6 (802.11ax)
//
// Network topology:
//
//    Station (STA)          Access Point (AP)
//      * <--    distance    -->  *
//      |                         |
//    node 0                    node 1
//
// Traffic is uplink (STA -> AP), packet size and interval are configurable

#include "ns3/attribute-container.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/data-processor.h"
#include "ns3/eht-configuration.h"
#include "ns3/eht-phy.h"
#include "ns3/frame-exchange-manager.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-server.h"
#include "ns3/qos-utils.h"
#include "ns3/spectrum-channel.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "json.hpp"

#include <regex>

#define PI 3.1415926535


using namespace ns3;
using json = nlohmann::json;

NS_LOG_COMPONENT_DEFINE("single-sta-single-link");

// Based on ConstantRate (m_dataMode and m_ctlMode are set to public for easy access)
class AiWifiManager : public WifiRemoteStationManager
{
public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::AiWifiManager")
                .SetParent<WifiRemoteStationManager>()
                .SetGroupName("Wifi")
                .AddConstructor<AiWifiManager>()
                .AddAttribute("DataMode",
                              "The transmission mode to use for every data packet transmission",
                              StringValue("OfdmRate6Mbps"),
                              MakeWifiModeAccessor(&AiWifiManager::m_dataMode),
                              MakeWifiModeChecker())
                .AddAttribute("ControlMode",
                              "The transmission mode to use for every RTS packet transmission.",
                              StringValue("OfdmRate6Mbps"),
                              MakeWifiModeAccessor(&AiWifiManager::m_ctlMode),
                              MakeWifiModeChecker());
        return tid;
    }

    AiWifiManager()
    {
    }

    ~AiWifiManager() override
    {
    }

    WifiMode m_dataMode; //!< Wifi mode for unicast Data frames
    WifiMode m_ctlMode;  //!< Wifi mode for RTS frames

private:
    WifiRemoteStation* DoCreateStation() const override
    {
        auto station = new WifiRemoteStation();
        return station;
    }

    void DoReportRxOk(WifiRemoteStation* station, double rxSnr, WifiMode txMode) override
    {
    }

    void DoReportRtsFailed(WifiRemoteStation* station) override
    {
    }

    void DoReportDataFailed(WifiRemoteStation* station) override
    {
    }

    void DoReportRtsOk(WifiRemoteStation* station,
                       double ctsSnr,
                       WifiMode ctsMode,
                       double rtsSnr) override
    {
    }

    void DoReportDataOk(WifiRemoteStation* station,
                        double ackSnr,
                        WifiMode ackMode,
                        double dataSnr,
                        MHz_u dataChannelWidth,
                        uint8_t dataNss) override
    {
    }

    void DoReportFinalRtsFailed(WifiRemoteStation* station) override
    {
    }

    void DoReportFinalDataFailed(WifiRemoteStation* station) override
    {
    }

    WifiTxVector DoGetDataTxVector(WifiRemoteStation* station, MHz_u allowedWidth) override
    {
        uint8_t nss = std::min(GetMaxNumberOfTransmitStreams(), GetNumberOfSupportedStreams(station));
        if (m_dataMode.GetModulationClass() == WIFI_MOD_CLASS_HT)
        {
            nss = 1 + (m_dataMode.GetMcsValue() / 8);
        }
        return WifiTxVector(
            m_dataMode,
            GetDefaultTxPowerLevel(),
            GetPreambleForTransmission(m_dataMode.GetModulationClass(), GetShortPreambleEnabled()),
            GetGuardIntervalForMode(m_dataMode,
                                    GetShortGuardIntervalSupported(station),
                                    GetGuardInterval(station)),
            GetNumberOfAntennas(),
            nss,
            0,
            GetPhy()->GetTxBandwidth(m_dataMode, std::min(allowedWidth, GetChannelWidth(station))),
            GetAggregation(station));
    }

    WifiTxVector DoGetRtsTxVector(WifiRemoteStation* station) override
    {
        return WifiTxVector(
            m_ctlMode,
            GetDefaultTxPowerLevel(),
            GetPreambleForTransmission(m_ctlMode.GetModulationClass(), GetShortPreambleEnabled()),
            GetGuardIntervalForMode(m_ctlMode,
                                    GetShortGuardIntervalSupported(station),
                                    GetGuardInterval(station)),
            1,
            1,
            0,
            GetPhy()->GetTxBandwidth(m_ctlMode, GetChannelWidth(station)),
            GetAggregation(station));
    }
};
NS_OBJECT_ENSURE_REGISTERED(AiWifiManager);

// Data processor (south bound)
auto dataProcessor = CreateObject<DataProcessor>();

Time measStartTime;
Time measInterval;
int actionWaitTimeMs;
Time stopTime;

WifiTxStatsHelper wifiTxStats;
uint32_t totalSucc = 0;
uint32_t totalFail = 0;
NetDeviceContainer staDevCon;
NetDeviceContainer apDevCon;
Ptr<AiWifiManager> aiManager;

std::pair<uint32_t, uint32_t>
GetTotalSuccAndFailCounts()
{
    const auto recvdPktsMap = wifiTxStats.GetSuccessesByNodeDeviceLink();
    const auto failedPktsMap = wifiTxStats.GetFailuresByNodeDevice();
    auto nodeDevLinkTuple = std::tuple<uint32_t, uint32_t, uint8_t>(0, 0, 0);
    auto nodeDevTuple = std::tuple<uint32_t, uint32_t>(0, 0);
    uint32_t nRecvd = recvdPktsMap.contains(nodeDevLinkTuple)
                          ? recvdPktsMap.at(nodeDevLinkTuple)
                          : 0;
    uint32_t nFailed = failedPktsMap.contains(nodeDevTuple)
                           ? failedPktsMap.at(nodeDevTuple)
                           : 0;
    return std::make_pair(nRecvd, nFailed);
}

void
GenerateMeasurement()
{
    if (aiManager == nullptr)
    {
        aiManager = DynamicCast<AiWifiManager>(
            DynamicCast<WifiNetDevice>(staDevCon.Get(0))->GetRemoteStationManager());
        std::cout << aiManager << std::endl;
    }
    auto totalCounts = GetTotalSuccAndFailCounts();

    std::cout << "at " << Simulator::Now().ToDouble(Time::MS) << " ms, " <<
        "measurement: succ(total)=" << std::get<0>(totalCounts) <<
            ", fail(total)=" << std::get<1>(totalCounts) <<
            ", succ(period)=" << std::get<0>(totalCounts) - totalSucc <<
            ", fail(period)=" << std::get<1>(totalCounts) - totalFail <<
            ", mcsUsed=" << +aiManager->m_dataMode.GetMcsValue() << std::endl;

    // Create one measurement
    auto meas = CreateObject<NetworkStats>("TsRateControl", 0, Simulator::Now().GetMilliSeconds());
    meas->Append("meas::succ", std::get<0>(totalCounts) - totalSucc);
    meas->Append("meas::fail", std::get<1>(totalCounts) - totalFail);
    // meas->Append("meas::mcsUsed", aiManager->m_dataMode.GetMcsValue());
    dataProcessor->AppendMeasurement(meas);

    // Update temp counter
    totalSucc = std::get<0>(totalCounts);
    totalFail = std::get<1>(totalCounts);

    Simulator::Schedule(measInterval, &GenerateMeasurement);
}

void
RecvAction(const json& action)
{
    if (action == nullptr)
    {
        return;
    }
    std::cout << "at " << Simulator::Now().ToDouble(Time::MS) << " ms, " <<
        "action: mcsNew=" << action.get<int>() << std::endl;

    // Update data and basic rates
    auto mcsString = "HeMcs" + std::to_string(action.get<int>());
    std::cout << mcsString << std::endl;
    aiManager->m_dataMode = WifiMode(mcsString);
    uint32_t nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(action.get<int>()) / 1e6;
    auto basicString = "OfdmRate" + std::to_string(nonHtRefRateMbps) + "Mbps";
    std::cout << basicString << std::endl;
    aiManager->m_ctlMode = WifiMode(basicString);
}

int
main(int argc, char* argv[])
{
    double frequency{5}; // 2.4 / 5 / 6 GHz
    int mcs{1};    // Initial mcs
    int channelWidth{20};
    dBm_u txPowerdBm{16.0};
    meter_u distance{30.0};
    dB_u noiseFigure{7.0};
    uint32_t payloadSize{1000};
    Time packetInterval{"0.1ms"};
    int64_t streamNumber{1};

    CommandLine cmd(__FILE__);
    cmd.AddValue("frequency", "", frequency);
    cmd.AddValue("mcs", "", mcs);
    cmd.AddValue("channelWidth", "", channelWidth);
    cmd.AddValue("txPowerdBm", "", txPowerdBm);
    cmd.AddValue("distance", "", distance);
    cmd.AddValue("noiseFigure", "", noiseFigure);
    cmd.AddValue("payloadSize", "", payloadSize);
    cmd.AddValue("packetInterval", "", packetInterval);
    cmd.AddValue("streamNumber", "", streamNumber);
    cmd.Parse(argc, argv);

    NodeContainer wifiStaNodeCon;
    wifiStaNodeCon.Create(1);
    NodeContainer wifiApNodeCon;
    wifiApNodeCon.Create(1);

    const std::string widthStr = std::to_string(channelWidth);
    std::string channelStr("{0, " + widthStr + ", ");
    StringValue ctrlRate;
    uint32_t nonHtRefRateMbps = HePhy::GetNonHtReferenceRate(mcs) / 1e6;
    std::ostringstream ossDataMode;
    ossDataMode << "HeMcs" << mcs;
    if (frequency == 6)
    {
        ctrlRate = StringValue(ossDataMode.str());
        channelStr += "BAND_6GHZ, 0}";
        Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                           DoubleValue(48));
    }
    else if (frequency == 5)
    {
        std::ostringstream ossControlMode;
        ossControlMode << "OfdmRate" << nonHtRefRateMbps << "Mbps";
        ctrlRate = StringValue(ossControlMode.str());
        channelStr += "BAND_5GHZ, 0}";
    }
    else if (frequency == 2.4)
    {
        std::ostringstream ossControlMode;
        ossControlMode << "ErpOfdmRate" << nonHtRefRateMbps << "Mbps";
        ctrlRate = StringValue(ossControlMode.str());
        channelStr += "BAND_2_4GHZ, 0}";
        Config::SetDefault("ns3::LogDistancePropagationLossModel::ReferenceLoss",
                           DoubleValue(40));
    }
    else
    {
        NS_FATAL_ERROR("Wrong frequency band value!");
    }

    // Set the noise figure (default is 7 dB) for Wi-Fi PHY
    Config::SetDefault("ns3::WifiPhy::RxNoiseFigure", DoubleValue(noiseFigure));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    wifi.SetRemoteStationManager("ns3::AiWifiManager",
                                 "DataMode",
                                 StringValue(ossDataMode.str()),
                                 "ControlMode",
                                 ctrlRate);

    YansWifiPhyHelper phy;
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    phy.Set("ChannelSettings", StringValue(channelStr));
    auto channelHelp = YansWifiChannelHelper();
    channelHelp.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    channelHelp.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    auto chn = channelHelp.Create();
    phy.SetChannel(chn);
    phy.Set("TxPowerStart", DoubleValue(txPowerdBm));
    phy.Set("TxPowerEnd", DoubleValue(txPowerdBm));

    WifiMacHelper mac;

    Ssid ssid = Ssid("single-sta-single-link-bss");
    mac.SetType("ns3::StaWifiMac",
                "Ssid",
                SsidValue(ssid));
    staDevCon = wifi.Install(phy, mac, wifiStaNodeCon);
    mac.SetType("ns3::ApWifiMac",
                "Ssid",
                SsidValue(ssid));
    apDevCon = wifi.Install(phy, mac, wifiApNodeCon);

    streamNumber += channelHelp.AssignStreams(chn, streamNumber);
    streamNumber += WifiHelper::AssignStreams(staDevCon, streamNumber);
    streamNumber += WifiHelper::AssignStreams(apDevCon, streamNumber);

    // mobility.
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    mobility.SetPositionAllocator(positionAlloc);
    positionAlloc->Add(Vector(0, 0, 0)); // STA
    positionAlloc->Add(Vector(0, distance, 0)); // AP
    mobility.Install(wifiStaNodeCon);
    mobility.Install(wifiApNodeCon);

    // Install packet socket
    PacketSocketHelper packetSocket;
    packetSocket.Install(wifiStaNodeCon);
    packetSocket.Install(wifiApNodeCon);

    // random start time
    Ptr<UniformRandomVariable> startTime = CreateObject<UniformRandomVariable>();
    startTime->SetStream(streamNumber);
    startTime->SetAttribute("Min", DoubleValue(0.0));
    startTime->SetAttribute("Max", DoubleValue(1.0));

    // setup PacketSocketServer for AP because the traffic is uplink
    PacketSocketAddress srvAddr;
    srvAddr.SetSingleDevice(DynamicCast<WifiNetDevice>(apDevCon.Get(0))->GetIfIndex());
    srvAddr.SetProtocol(1);
    auto psServer = CreateObject<PacketSocketServer>();
    psServer->SetLocal(srvAddr);
    apDevCon.Get(0)->GetNode()->AddApplication(psServer);
    psServer->SetStartTime(Seconds(0)); // server starts at 0 s

    // config uplink traffic for STA
    auto apAddr = apDevCon.Get(0)->GetAddress();
    PacketSocketAddress sockAddr;
    sockAddr.SetSingleDevice(staDevCon.Get(0)->GetIfIndex());
    sockAddr.SetPhysicalAddress(apAddr);
    sockAddr.SetProtocol(1);
    auto tid = wifiAcList.at(AC_BE).GetLowTid();
    auto client = CreateObject<PacketSocketClient>();
    client->SetAttribute("PacketSize", UintegerValue(payloadSize));
    client->SetAttribute("MaxPackets", UintegerValue(0));
    client->SetAttribute("Interval", TimeValue(packetInterval));
    client->SetAttribute("Priority", UintegerValue(tid));
    client->SetRemote(sockAddr);
    client->SetStartTime(Seconds(startTime->GetValue()));
    wifiStaNodeCon.Get(0)->AddApplication(client);

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
    dataProcessor->SetNetworkGymActionCallback("TsRateControl::mcsNew", 0, MakeCallback(&RecvAction));

    // TX stats
    wifiTxStats.Enable(staDevCon);
    wifiTxStats.Enable(apDevCon);
    wifiTxStats.Start(Seconds(1));
    wifiTxStats.Stop(stopTime + Seconds(1));

    Simulator::Stop(stopTime + Seconds(1));
    Simulator::Run();

    // calculate PER
    const auto recvdPktsMap = wifiTxStats.GetSuccessesByNodeDeviceLink();
    const auto failedPktsMap = wifiTxStats.GetFailuresByNodeDevice();
    auto nodeDevLinkTuple = std::tuple<uint32_t, uint32_t, uint8_t>(0, 0, 0);
    auto nodeDevTuple = std::tuple<uint32_t, uint32_t>(0, 0);
    uint64_t nRecvd = recvdPktsMap.contains(nodeDevLinkTuple)
                          ? recvdPktsMap.at(nodeDevLinkTuple)
                          : 0;
    uint64_t nFailed = failedPktsMap.contains(nodeDevTuple)
                           ? failedPktsMap.at(nodeDevTuple)
                           : 0;
    double per = static_cast<double>(nFailed) / (nFailed + nRecvd);

    std::cout << "frequency" << "," << "mcs" << "," << "channelWidth" << "," << "txPowerdBm" << "," <<
    "distance" << "," << "noiseFigure" << "," << "payloadSize" << "," << "packetInterval" << ","
    << "nTotal" << "," << "nRecvd" << "," << "nFailed" << "," << "per" << std::endl;
    std::cout << frequency << "," << mcs << "," << channelWidth << "," << txPowerdBm << "," <<
        distance << "," << noiseFigure << "," << payloadSize << "," << packetInterval.
        GetMilliSeconds() << "," << nRecvd + nFailed << "," << nRecvd << "," << nFailed << "," <<
        per << std::endl;

    Simulator::Destroy();
    return 0;
}
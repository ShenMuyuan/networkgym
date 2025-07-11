#include "json.hpp"

#include "ns3/ap-wifi-mac.h"
#include "ns3/buildings-module.h"
#include "ns3/burst-sink-helper.h"
#include "ns3/bursty-helper.h"
#include "ns3/core-module.h"
#include "ns3/data-processor.h"
#include "ns3/double.h"
#include "ns3/frame-exchange-manager.h"
#include "ns3/he-phy.h"
#include "ns3/integer.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-building-info.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-socket-client.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-server.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/qos-txop.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/ssid.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/string.h"
#include "ns3/threshold-preamble-detection-model.h"
#include "ns3/traced-value.h"
#include "ns3/traffic-control-layer.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-remote-station-manager.h"
#include "ns3/wifi-tx-stats-helper.h"
#include "ns3/yans-wifi-helper.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace ns3;
using json = nlohmann::json;

/// Avoid std::numbers::pi because it's C++20
#define PI 3.1415926535
#define N_BSS 4

/**
 * \ingroup wifi
 *
 * The TgaxResidentialPropagationLossModel in ns3-ai repo.
 */
class TgaxResidentialPropagationLossModel : public PropagationLossModel
{
public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::TgaxResidentialPropagationLossModel")
            .SetParent<PropagationLossModel>()
            .SetGroupName("Wifi")
            .AddConstructor<TgaxResidentialPropagationLossModel>()
            .AddAttribute("Frequency",
                          "The carrier frequency (in Hz) at which propagation occurs",
                          DoubleValue(2.437e9),
                          MakeDoubleAccessor(&TgaxResidentialPropagationLossModel::m_frequencyHz),
                          MakeDoubleChecker<double>())
            .AddAttribute(
                "ShadowSigma",
                "Standard deviation (dB) of the normal distribution used to calculate shadowing "
                "loss",
                DoubleValue(5.0),
                MakeDoubleAccessor(&TgaxResidentialPropagationLossModel::m_shadowingSigma),
                MakeDoubleChecker<double>());
        return tid;
    }

    TgaxResidentialPropagationLossModel()
    {
        m_shadowingRandomVariable = CreateObject<NormalRandomVariable>();
    }

    double GetRxPower(dBm_u txPowerDbm,
                      const Ptr<MobilityModel>& a,
                      const Ptr<MobilityModel>& b) const
    {
        // Modified to avoid redundant code
        return DoCalcRxPower(txPowerDbm, a, b);
    }

protected:
    double DoCalcRxPower(double txPowerDbm,
                         Ptr<MobilityModel> a,
                         Ptr<MobilityModel> b) const override
    {
        double distance = a->GetDistanceFrom(b);

        if (distance == 0)
        {
            return txPowerDbm;
        }

        distance = std::max(1.0, distance); // 1m minimum distance
        double pathlossDb;
        double breakpointDistance = 5; // meters
        double fc = 2.4e9; // carrier frequency, Hz
        uint16_t floors = 0;
        uint16_t walls = 0;
        Ptr<MobilityBuildingInfo> aInfo = a->GetObject<MobilityBuildingInfo>();
        Ptr<MobilityBuildingInfo> bInfo = b->GetObject<MobilityBuildingInfo>();
        if (aInfo && bInfo)
        {
            if (!aInfo->IsIndoor() || !bInfo->IsIndoor())
            {
                NS_LOG_DEBUG("One or both nodes is outdoor, so returning zero signal power");
                return 0;
            }
            floors = std::abs(aInfo->GetFloorNumber() - bInfo->GetFloorNumber());
            walls = std::abs(aInfo->GetRoomNumberX() - bInfo->GetRoomNumberX()) +
                    std::abs(aInfo->GetRoomNumberY() - bInfo->GetRoomNumberY());
        }

        pathlossDb = 40.05 + 20 * std::log10(m_frequencyHz / fc) +
                     20 * std::log10(std::min(distance, breakpointDistance));
        if (distance > breakpointDistance)
        {
            pathlossDb += 35 * std::log10(distance / 5);
        }
        if (floors)
        {
            pathlossDb +=
                18.3 * std::pow((distance / floors),
                                ((distance / floors) + 2.0) / ((distance / floors) + 1.0) - 0.46);
        }
        if (walls)
        {
            pathlossDb += 5.0 * (walls); // Changed (distance/walls) to only (walls) because the
            // pathloss would isolate the rooms
        }

        return txPowerDbm - pathlossDb;
    }

    int64_t DoAssignStreams(int64_t stream) override
    {
        m_shadowingRandomVariable->SetStream(stream);
        return 1;
    }

private:
    double m_frequencyHz; //!< frequency, in Hz
    double m_shadowingSigma; //!< sigma (dB) for shadowing std. deviation
    Ptr<NormalRandomVariable>
    m_shadowingRandomVariable; //!< random variable used for shadowing loss
};

NS_OBJECT_ENSURE_REGISTERED(TgaxResidentialPropagationLossModel);

struct AutoMcsWifiRemoteStation : public WifiRemoteStation
{
    double m_lastSnrObserved; //!< SNR of most recently reported packet sent to the remote station
    uint16_t m_lastChannelWidthObserved;
    //!< Channel width (in MHz) of most recently reported
       //!< packet sent to the remote station
    uint16_t m_lastNssObserved;
    //!<  Number of spatial streams of most recently reported packet
       //!<  sent to the remote station
    double m_lastSnrCached; //!< SNR most recently used to select a rate
    uint8_t m_lastNss; //!< Number of spatial streams most recently used to the remote station
    WifiMode m_lastMode; //!< Mode most recently used to the remote station
    uint16_t
    m_lastChannelWidth; //!< Channel width (in MHz) most recently used to the remote station
};

static constexpr double CACHE_INITIAL_VALUE = -100;

class AutoMcsWifiManager : public WifiRemoteStationManager
{
public:
    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::AutoMcsWifiManager")
            .SetParent<WifiRemoteStationManager>()
            .SetGroupName("Wifi")
            .AddConstructor<AutoMcsWifiManager>()
            .AddAttribute("BerThreshold",
                          "The maximum Bit Error Rate acceptable at any transmission mode",
                          DoubleValue(1e-7),
                          // This default value was modified
                          MakeDoubleAccessor(&AutoMcsWifiManager::m_ber),
                          MakeDoubleChecker<double>())
            .AddAttribute("autoMCS",
                          "If enabled, select the best MCS for each STA-AP pair given the SNR.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&AutoMcsWifiManager::m_autoMCS),
                          MakeBooleanChecker())
            .AddTraceSource("Rate",
                            "Traced value for rate changes (b/s)",
                            MakeTraceSourceAccessor(&AutoMcsWifiManager::m_currentRate),
                            "ns3::TracedValueCallback::Uint64");
        return tid;
    }

    AutoMcsWifiManager()
        : m_currentRate(0)
    {
    }

    ~AutoMcsWifiManager() override
    {
    }

    void SetupPhy(const Ptr<WifiPhy> phy) override
    {
        WifiRemoteStationManager::SetupPhy(phy);
    }

private:
    void DoInitialize() override
    {
        BuildSnrThresholds();
        WifiRemoteStationManager::DoInitialize();
    }

    WifiRemoteStation* DoCreateStation() const override
    {
        auto* station = new AutoMcsWifiRemoteStation();
        Reset(station);
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

    void DoReportRtsOk(WifiRemoteStation* st,
                       double ctsSnr,
                       WifiMode ctsMode,
                       double rtsSnr) override
    {
        auto* station = static_cast<AutoMcsWifiRemoteStation*>(st);
        station->m_lastSnrObserved = rtsSnr;
        station->m_lastChannelWidthObserved =
            GetPhy()->GetChannelWidth() >= 40 ? 20 : GetPhy()->GetChannelWidth();
        station->m_lastNssObserved = 1;
    }

    void DoReportDataOk(WifiRemoteStation* st,
                        double ackSnr,
                        WifiMode ackMode,
                        double dataSnr,
                        MHz_u dataChannelWidth,
                        uint8_t dataNss) override
    {
        auto* station = static_cast<AutoMcsWifiRemoteStation*>(st);
        if (dataSnr == 0)
        {
            NS_LOG_WARN("DataSnr reported to be zero; not saving this report.");
            return;
        }
        station->m_lastSnrObserved = dataSnr;
        station->m_lastChannelWidthObserved = dataChannelWidth;
        station->m_lastNssObserved = dataNss;
        if (station->m_lastMode != GetDefaultMode())
        {
            choosenMCS.emplace_back(station->m_lastMode.GetMcsValue());
        }
    }

    void DoReportAmpduTxStatus(WifiRemoteStation* st,
                               uint16_t nSuccessfulMpdus,
                               uint16_t nFailedMpdus,
                               double rxSnr,
                               double dataSnr,
                               MHz_u dataChannelWidth,
                               uint8_t dataNss) override
    {
        auto* station = static_cast<AutoMcsWifiRemoteStation*>(st);
        if (dataSnr == 0)
        {
            NS_LOG_WARN("DataSnr reported to be zero; not saving this report.");
            return;
        }
        for (int i = 0; i < nSuccessfulMpdus; i++)
        {
            choosenMCS.emplace_back(station->m_lastMode.GetMcsValue());
        }
        station->m_lastSnrObserved = dataSnr;
        station->m_lastChannelWidthObserved = dataChannelWidth;
        station->m_lastNssObserved = dataNss;
    }

    void DoReportFinalRtsFailed(WifiRemoteStation* station) override
    {
        Reset(station);
    }

    void DoReportFinalDataFailed(WifiRemoteStation* station) override
    {
        Reset(station);
    }

    WifiTxVector DoGetDataTxVector(WifiRemoteStation* st, MHz_u allowedWidth) override
    {
        auto* station = static_cast<AutoMcsWifiRemoteStation*>(st);
        // We search within the Supported rate set the mode with the
        // highest data rate for which the SNR threshold is smaller than m_lastSnr
        // to ensure correct packet delivery.
        WifiMode maxMode = GetDefaultModeForSta(st);
        WifiTxVector txVector;
        WifiMode mode;
        uint64_t bestRate = 0;
        uint8_t selectedNss = 1;
        uint16_t guardInterval;
        uint16_t channelWidth = std::min(GetChannelWidth(station), allowedWidth);
        txVector.SetChannelWidth(channelWidth);

        if ((Simulator::Now().GetSeconds() < 10))
        {
            if ((station->m_lastSnrCached != CACHE_INITIAL_VALUE) &&
                (station->m_lastSnrObserved == station->m_lastSnrCached) &&
                (channelWidth == station->m_lastChannelWidth))
            {
                // SNR has not changed, so skip the search and use the last mode selected
                maxMode = station->m_lastMode;
                selectedNss = station->m_lastNss;
                NS_LOG_DEBUG(
                    "Using cached mode = " << maxMode.GetUniqueName() << " last snr observed "
                    << station->m_lastSnrObserved << " cached "
                    << station->m_lastSnrCached << " channel width "
                    << station->m_lastChannelWidth << " nss "
                    << +selectedNss);
            }

            else
            {
                if (GetHtSupported() && GetHtSupported(st))
                {
                    for (uint8_t i = 0; i < GetNMcsSupported(station); i++)
                    {
                        mode = GetMcsSupported(station, i);
                        txVector.SetMode(mode);
                        if (mode.GetModulationClass() == WIFI_MOD_CLASS_HT)
                        {
                            guardInterval = static_cast<uint16_t>(
                                std::max(GetShortGuardIntervalSupported(station) ? 400 : 800,
                                         GetShortGuardIntervalSupported() ? 400 : 800));
                            txVector.SetGuardInterval(NanoSeconds(guardInterval));
                            // If the node and peer are both VHT capable, only search VHT modes
                            if (GetVhtSupported() && GetVhtSupported(station))
                            {
                                continue;
                            }
                            // If the node and peer are both HE capable, only search HE modes
                            if (GetHeSupported() && GetHeSupported(station))
                            {
                                continue;
                            }
                            // Derive NSS from the MCS index. There is a different mode for each
                            // possible NSS value.
                            uint8_t nss = (mode.GetMcsValue() / 8) + 1;
                            txVector.SetNss(nss);
                            if (!txVector.IsValid() || nss > std::min(
                                    GetMaxNumberOfTransmitStreams(),
                                    GetNumberOfSupportedStreams(st)))
                            {
                                NS_LOG_DEBUG(
                                    "Skipping mode " << mode.GetUniqueName() << " nss " << +nss
                                    << " width "
                                    << txVector.GetChannelWidth());
                                continue;
                            }
                            double threshold = GetSnrThreshold(txVector);
                            uint64_t dataRate = mode.GetDataRate(txVector.GetChannelWidth(),
                                                                 txVector.GetGuardInterval(),
                                                                 nss);
                            NS_LOG_DEBUG("Testing mode " << mode.GetUniqueName() << " data rate "
                                << dataRate << " threshold " << threshold
                                << " last snr observed "
                                << station->m_lastSnrObserved << " cached "
                                << station->m_lastSnrCached);
                            double snr = GetLastObservedSnr(station, channelWidth, nss);
                            if (dataRate > bestRate && threshold < snr)
                            {
                                NS_LOG_DEBUG("Candidate mode = "
                                    << mode.GetUniqueName() << " data rate " << dataRate
                                    << " threshold " << threshold << " channel width "
                                    << channelWidth << " snr " << snr);
                                bestRate = dataRate;
                                maxMode = mode;
                                selectedNss = nss;
                            }
                        }
                        else if (mode.GetModulationClass() == WIFI_MOD_CLASS_VHT)
                        {
                            guardInterval = static_cast<uint16_t>(
                                std::max(GetShortGuardIntervalSupported(station) ? 400 : 800,
                                         GetShortGuardIntervalSupported() ? 400 : 800));
                            txVector.SetGuardInterval(NanoSeconds(guardInterval));
                            // If the node and peer are both HE capable, only search HE modes
                            if (GetHeSupported() && GetHeSupported(station))
                            {
                                continue;
                            }
                            // If the node and peer are not both VHT capable, only search HT modes
                            if (!GetVhtSupported() || !GetVhtSupported(station))
                            {
                                continue;
                            }
                            for (uint8_t nss = 1; nss <= std::min(GetMaxNumberOfTransmitStreams(),
                                                      GetNumberOfSupportedStreams(station));
                                 nss++)
                            {
                                txVector.SetNss(nss);
                                if (!txVector.IsValid())
                                {
                                    NS_LOG_DEBUG("Skipping mode " << mode.GetUniqueName() << " nss "
                                        << +nss << " width "
                                        << txVector.GetChannelWidth());
                                    continue;
                                }
                                double threshold = GetSnrThreshold(txVector);
                                uint64_t dataRate = mode.GetDataRate(txVector.GetChannelWidth(),
                                    txVector.GetGuardInterval(),
                                    nss);
                                NS_LOG_DEBUG("Testing mode = "
                                    << mode.GetUniqueName() << " data rate " << dataRate
                                    << " threshold " << threshold << " last snr observed "
                                    << station->m_lastSnrObserved << " cached "
                                    << station->m_lastSnrCached);
                                double snr = GetLastObservedSnr(station, channelWidth, nss);
                                if (dataRate > bestRate && threshold < snr)
                                {
                                    NS_LOG_DEBUG("Candidate mode = " << mode.GetUniqueName()
                                        << " data rate " << dataRate
                                        << " channel width "
                                        << channelWidth << " snr " << snr);
                                    bestRate = dataRate;
                                    maxMode = mode;
                                    selectedNss = nss;
                                }
                            }
                        }
                        else // HE
                        {
                            guardInterval = std::max(GetGuardInterval(station).ToInteger(Time::NS),
                                                     GetGuardInterval().ToInteger(Time::NS));
                            txVector.SetGuardInterval(NanoSeconds(guardInterval));
                            // If the node and peer are not both HE capable, only search (V)HT modes
                            if (!GetHeSupported() || !GetHeSupported(station))
                            {
                                continue;
                            }
                            for (uint8_t nss = 1; nss <= std::min(GetMaxNumberOfTransmitStreams(),
                                                      GetNumberOfSupportedStreams(station));
                                 nss++)
                            {
                                txVector.SetNss(nss);
                                if (!txVector.IsValid())
                                {
                                    NS_LOG_DEBUG("Skipping mode " << mode.GetUniqueName() << " nss "
                                        << +nss << " width "
                                        << +txVector.GetChannelWidth());
                                    continue;
                                }
                                double threshold = GetSnrThreshold(txVector);
                                uint64_t dataRate = mode.GetDataRate(txVector.GetChannelWidth(),
                                    txVector.GetGuardInterval(),
                                    nss);
                                NS_LOG_DEBUG("Testing mode = "
                                    << mode.GetUniqueName() << " data rate " << dataRate
                                    << " threshold " << threshold << " last snr observed "
                                    << station->m_lastSnrObserved << " cached "
                                    << station->m_lastSnrCached);
                                double snr = GetLastObservedSnr(station, channelWidth, nss);
                                if (dataRate > bestRate && threshold < snr)
                                {
                                    NS_LOG_DEBUG("Candidate mode = "
                                        << mode.GetUniqueName() << " data rate " << dataRate
                                        << " threshold " << threshold << " channel width "
                                        << channelWidth << " snr " << snr);
                                    bestRate = dataRate;
                                    maxMode = mode;
                                    selectedNss = nss;
                                }
                            }
                        }
                    }
                }
                else
                {
                    // Non-HT selection
                    selectedNss = 1;
                    for (uint8_t i = 0; i < GetNSupported(station); i++)
                    {
                        mode = GetSupported(station, i);
                        txVector.SetMode(mode);
                        txVector.SetNss(selectedNss);
                        uint16_t width = GetChannelWidthForNonHtMode(mode);
                        txVector.SetChannelWidth(width);
                        double threshold = GetSnrThreshold(txVector);
                        uint64_t dataRate = mode.GetDataRate(txVector.GetChannelWidth(),
                                                             txVector.GetGuardInterval(),
                                                             txVector.GetNss());
                        NS_LOG_DEBUG("mode = " << mode.GetUniqueName() << " threshold " << threshold
                            << " last snr observed " << station->m_lastSnrObserved);
                        double snr = GetLastObservedSnr(station, width, 1);
                        if (dataRate > bestRate && threshold < snr)
                        {
                            NS_LOG_DEBUG(
                                "Candidate mode = " << mode.GetUniqueName() << " data rate "
                                << dataRate << " threshold " << threshold
                                << " snr " << snr);
                            bestRate = dataRate;
                            maxMode = mode;
                        }
                    }
                }
                NS_LOG_DEBUG("Updating cached values for station to "
                    << maxMode.GetUniqueName() << " snr " << station->m_lastSnrObserved);
                station->m_lastSnrCached = station->m_lastSnrObserved;
                station->m_lastMode = maxMode;
                station->m_lastNss = selectedNss;
            }
        }
        else
        {
            double average = 0;
            for (auto it : choosenMCS)
            {
                average += it;
            }
            // std::cout << " raw value " << average / choosenMCS.size() << std::endl;
            average = std::ceil(average / choosenMCS.size());
            std::string mcs = "HeMcs" + std::to_string(static_cast<int>(average));
            maxMode = WifiMode(mcs);
        }
        NS_LOG_DEBUG("Found maxMode: " << maxMode << " channelWidth: " << channelWidth
            << " nss: " << +selectedNss);
        station->m_lastChannelWidth = channelWidth;
        if (maxMode.GetModulationClass() == WIFI_MOD_CLASS_HE)
        {
            guardInterval = std::max(GetGuardInterval(station).ToInteger(Time::NS),
                                     GetGuardInterval().ToInteger(Time::NS));
        }
        else if ((maxMode.GetModulationClass() == WIFI_MOD_CLASS_HT) ||
                 (maxMode.GetModulationClass() == WIFI_MOD_CLASS_VHT))
        {
            guardInterval =
                static_cast<uint16_t>(std::max(GetShortGuardIntervalSupported(station) ? 400 : 800,
                                               GetShortGuardIntervalSupported() ? 400 : 800));
        }
        else
        {
            guardInterval = 800;
        }
        WifiTxVector bestTxVector{
            maxMode,
            GetDefaultTxPowerLevel(),
            GetPreambleForTransmission(maxMode.GetModulationClass(), GetShortPreambleEnabled()),
            NanoSeconds(guardInterval),
            GetNumberOfAntennas(),
            selectedNss,
            0,
            GetPhy()->GetTxBandwidth(maxMode, channelWidth),
            GetAggregation(station)};

        uint64_t maxDataRate = maxMode.GetDataRate(bestTxVector);

        if (m_currentRate != maxDataRate)
        {
            // std::cout << "New datarate: " << maxMode << std::endl;

            m_currentRate = maxDataRate;
        }

        return bestTxVector;
    }

    WifiTxVector DoGetRtsTxVector(WifiRemoteStation* st) override
    {
        if (!m_autoMCS)
        {
            auto* station = static_cast<AutoMcsWifiRemoteStation*>(st);
            // We search within the Basic rate set the mode with the highest
            // SNR threshold possible which is smaller than m_lastSnr to
            // ensure correct packet delivery.
            double maxThreshold = 0.0;
            WifiTxVector txVector;
            WifiMode mode;
            uint8_t nss = 1;
            WifiMode maxMode = GetDefaultMode();
            // RTS is sent in a non-HT frame
            for (uint8_t i = 0; i < GetNBasicModes(); i++)
            {
                mode = GetBasicMode(i);
                txVector.SetMode(mode);
                txVector.SetNss(nss);
                txVector.SetChannelWidth(GetChannelWidthForNonHtMode(mode));
                double threshold = GetSnrThreshold(txVector);
                if (threshold > maxThreshold && threshold < station->m_lastSnrObserved)
                {
                    maxThreshold = threshold;
                    maxMode = mode;
                }
            }
            return WifiTxVector(
                maxMode,
                GetDefaultTxPowerLevel(),
                GetPreambleForTransmission(maxMode.GetModulationClass(), GetShortPreambleEnabled()),
                NanoSeconds(800),
                GetNumberOfAntennas(),
                nss,
                0,
                GetChannelWidthForNonHtMode(maxMode),
                GetAggregation(station));
        }
        else
        {
            return WifiTxVector(
                WifiMode("OfdmRate6Mbps"),
                GetDefaultTxPowerLevel(),
                GetPreambleForTransmission(WifiMode("OfdmRate6Mbps").GetModulationClass(),
                                           GetShortPreambleEnabled()),
                GetGuardInterval(st),
                1,
                1,
                0,
                GetPhy()->GetTxBandwidth(WifiMode("OfdmRate6Mbps"), GetChannelWidth(st)),
                GetAggregation(st));
        }
    }

    void Reset(WifiRemoteStation* station) const
    {
        auto* st = static_cast<AutoMcsWifiRemoteStation*>(station);
        st->m_lastSnrObserved = 0.0;
        st->m_lastChannelWidthObserved = 0;
        st->m_lastNssObserved = 1;
        st->m_lastSnrCached = CACHE_INITIAL_VALUE;
        st->m_lastMode = GetDefaultMode();
        st->m_lastChannelWidth = 0;
        st->m_lastNss = 1;
    }

    void BuildSnrThresholds()
    {
        m_thresholds.clear();
        WifiTxVector txVector;
        uint8_t nss = 1;
        for (const auto& mode : GetPhy()->GetModeList())
        {
            txVector.SetChannelWidth(GetChannelWidthForNonHtMode(mode));
            txVector.SetNss(nss);
            txVector.SetMode(mode);
            NS_LOG_DEBUG("Adding mode = " << mode.GetUniqueName());
            AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
        }
        // Add all MCSes
        if (GetHtSupported())
        {
            for (const auto& mode : GetPhy()->GetMcsList())
            {
                for (uint16_t j = 20; j <= GetPhy()->GetChannelWidth(); j *= 2)
                {
                    txVector.SetChannelWidth(j);
                    if (mode.GetModulationClass() == WIFI_MOD_CLASS_HT)
                    {
                        uint16_t guardInterval = GetShortGuardIntervalSupported() ? 400 : 800;
                        txVector.SetGuardInterval(NanoSeconds(guardInterval));
                        // derive NSS from the MCS index
                        nss = (mode.GetMcsValue() / 8) + 1;
                        NS_LOG_DEBUG(
                            "Adding mode = " << mode.GetUniqueName() << " channel width " << j
                            << " nss " << +nss << " GI " << guardInterval);
                        txVector.SetNss(nss);
                        txVector.SetMode(mode);
                        AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
                    }
                    else // VHT or HE
                    {
                        uint16_t guardInterval;
                        if (mode.GetModulationClass() == WIFI_MOD_CLASS_VHT)
                        {
                            guardInterval = GetShortGuardIntervalSupported() ? 400 : 800;
                        }
                        else
                        {
                            guardInterval = GetGuardInterval().ToInteger(Time::NS);
                        }
                        txVector.SetGuardInterval(NanoSeconds(guardInterval));
                        for (uint8_t k = 1; k <= GetPhy()->GetMaxSupportedTxSpatialStreams(); k++)
                        {
                            if (mode.IsAllowed(j, k))
                            {
                                NS_LOG_DEBUG("Adding mode = " << mode.GetUniqueName()
                                    << " channel width " << j << " nss " << +k
                                    << " GI " << guardInterval);
                                txVector.SetNss(k);
                                txVector.SetMode(mode);
                                AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
                                std::vector<double> berThreshold{8.37e-7};
                            }
                            else
                            {
                                NS_LOG_DEBUG("Mode = " << mode.GetUniqueName() << " disallowed");
                            }
                        }
                    }
                }
            }
        }
    }

    double GetSnrThreshold(WifiTxVector txVector)
    {
        auto it = std::find_if(m_thresholds.begin(),
                               m_thresholds.end(),
                               [&txVector](const std::pair<double, WifiTxVector>& p) -> bool {
                                   return ((txVector.GetMode() == p.second.GetMode()) &&
                                           (txVector.GetNss() == p.second.GetNss()) &&
                                           (txVector.GetChannelWidth() == p.second.
                                            GetChannelWidth()));
                               });
        if (it == m_thresholds.end())
        {
            // This means capabilities have changed in runtime, hence rebuild SNR thresholds
            BuildSnrThresholds();
            it = std::find_if(m_thresholds.begin(),
                              m_thresholds.end(),
                              [&txVector](const std::pair<double, WifiTxVector>& p) -> bool {
                                  return ((txVector.GetMode() == p.second.GetMode()) &&
                                          (txVector.GetNss() == p.second.GetNss()) &&
                                          (txVector.GetChannelWidth() == p.second.
                                           GetChannelWidth()));
                              });
            NS_ASSERT_MSG(it != m_thresholds.end(), "SNR threshold not found");
        }
        return it->first;
    }

    void AddSnrThreshold(WifiTxVector txVector, double snr)
    {
        m_thresholds.emplace_back(snr, txVector);
    }

    static uint16_t GetChannelWidthForNonHtMode(WifiMode mode)
    {
        NS_ASSERT(mode.GetModulationClass() != WIFI_MOD_CLASS_HT &&
            mode.GetModulationClass() != WIFI_MOD_CLASS_VHT &&
            mode.GetModulationClass() != WIFI_MOD_CLASS_HE);
        if (mode.GetModulationClass() == WIFI_MOD_CLASS_DSSS ||
            mode.GetModulationClass() == WIFI_MOD_CLASS_HR_DSSS)
        {
            return 22;
        }
        else
        {
            return 20;
        }
    }

    static double GetLastObservedSnr(const AutoMcsWifiRemoteStation* station,
                                     uint16_t channelWidth,
                                     uint8_t nss)
    {
        double snr = station->m_lastSnrObserved;
        if (channelWidth != station->m_lastChannelWidthObserved)
        {
            snr /= (static_cast<double>(channelWidth) / station->m_lastChannelWidthObserved);
        }
        if (nss != station->m_lastNssObserved)
        {
            snr /= (static_cast<double>(nss) / station->m_lastNssObserved);
        }
        NS_LOG_DEBUG("Last observed SNR is " << station->m_lastSnrObserved << " for channel width "
            << station->m_lastChannelWidthObserved << " and nss "
            << +station->m_lastNssObserved << "; computed SNR is "
            << snr << " for channel width " << channelWidth
            << " and nss " << +nss);
        return snr;
    }

    typedef std::vector<std::pair<double, WifiTxVector>> Thresholds;
    double m_ber; //!< The maximum Bit Error Rate acceptable at any transmission mode
    Thresholds m_thresholds; //!< List of WifiTxVector and the minimum SNR pair
    TracedValue<uint64_t> m_currentRate; //!< Trace rate changes
    std::vector<int> choosenMCS;
    bool m_autoMCS; //!< Enable constant rate after a while
};

NS_OBJECT_ENSURE_REGISTERED(AutoMcsWifiManager);

Ptr<UniformRandomVariable> randomX = CreateObject<UniformRandomVariable>();
Ptr<UniformRandomVariable> randomY = CreateObject<UniformRandomVariable>();

double distance = 0.001; ///< The distance in meters between the AP and the STAs
uint8_t boxSize = 25;
std::map<int, std::string> configuration;
uint32_t pktSize = 1500; ///< packet size used for the simulation (in bytes)
uint8_t maxMpdus = 5; ///< The maximum number of MPDUs in A-MPDUs (0 to disable MPDU aggregation)

uint32_t networkSize;
NetDeviceContainer apDevices;
NetDeviceContainer staDevices;
NetDeviceContainer devices;
NodeContainer wifiNodes;
NodeContainer apNodes;
NodeContainer staNodes;
int apNodeCount = 4;
double txPower; ///< The transmit power of all the nodes in dBm
std::string propagationModel = "tgax";

std::unordered_map<uint64_t, int> bssOfNode;

std::map<uint32_t, std::vector<double>> nodeCw;
std::map<uint32_t, std::vector<double>> nodeBackoff;
std::map<uint64_t, int> dataRateToMcs;
std::map<uint32_t, int> nodeMcs;

std::vector<std::string>
csv_split(const std::string& source, char delimeter)
{
    std::vector<std::string> ret;
    std::string word = "";

    bool inQuote = false;
    for (uint32_t i = 0; i < source.size(); ++i)
    {
        if (!inQuote && source[i] == '"')
        {
            inQuote = true;
            continue;
        }
        if (inQuote && source[i] == '"')
        {
            if (source.size() > i && source[i + 1] == '"')
            {
                ++i;
            }
            else
            {
                inQuote = false;
                continue;
            }
        }

        if (!inQuote && source[i] == delimeter)
        {
            ret.push_back(word);
            word = "";
        }
        else
        {
            word += source[i];
        }
    }
    ret.push_back(word);

    return ret;
}

std::map<uint32_t, std::map<uint32_t, double>> nodeRxPower;

void
GetRxPower(Ptr<TgaxResidentialPropagationLossModel> tgaxPropModel)
{
    for (uint32_t i = 0; i < wifiNodes.GetN(); i++) // TX node
    {
        Ptr<NetDevice> dev = wifiNodes.Get(i)->GetDevice(0);
        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
        Ptr<Object> object = wifiNodes.Get(i);
        Ptr<MobilityModel> model1 = object->GetObject<MobilityModel>();
        Ptr<WifiPhy> wifi_phy = wifi_dev->GetPhy();

        for (uint32_t x = 0; x < wifiNodes.GetN(); x++) // RX node (must be in BSS-0)
        {
            // Skip same nodes
            if (i == x)
            {
                continue;
            }

            // Check RX node BSS
            if (auto nodeId = wifiNodes.Get(x)->GetId(); bssOfNode[nodeId] != 0)
            {
                continue;
            }

            Ptr<Object> object2 = wifiNodes.Get(x);
            Ptr<MobilityModel> model2 = object2->GetObject<MobilityModel>();
            double rxPower = 0;
            for (int j = 0; j < 100; j++)
            {
                rxPower += tgaxPropModel->GetRxPower(wifi_phy->GetTxPowerStart(), model1, model2);
            }
            nodeRxPower[wifiNodes.Get(i)->GetId()][wifiNodes.Get(x)->GetId()] = (rxPower / 100);
        }
    }
}

// Data processor (south bound)
auto dataProcessor = CreateObject<DataProcessor>();

Time measStartTime;
Time measInterval;
int actionWaitTimeMs;
Time stopTime;

WifiTxStatsHelper wifiTxStats;
std::map<uint32_t, uint32_t> stepSuccPerNode;
bool stepSuccPerNodeInitialized = false;

void
GenerateMeasurement()
{
    Ptr<TgaxResidentialPropagationLossModel> propModel =
        CreateObject<TgaxResidentialPropagationLossModel>();
    GetRxPower(propModel);

    // Default value of access delay, if no successful record
    double vrAccessDelayMs = measInterval.ToDouble(Time::MS);

    if (!stepSuccPerNodeInitialized)
    {
        for (auto i = 0; i < wifiNodes.GetN(); ++i)
        {
            stepSuccPerNode[i] = 0;
        }
        stepSuccPerNodeInitialized = true;
    }
    else
    {
        const auto succPktsMap = wifiTxStats.GetSuccessesByNodeDeviceLink();
        const auto successRecords = wifiTxStats.GetSuccessRecords();
        for (auto i = 0; i < wifiNodes.GetN(); ++i)
        {
            if (i < N_BSS)  // APs
            {
                continue;
            }
            auto nodeDevLinkTuple = std::tuple<uint32_t, uint32_t, uint8_t>(i, 0, 0);
            uint32_t nRecvdTotal = succPktsMap.contains(nodeDevLinkTuple)
                  ? succPktsMap.at(nodeDevLinkTuple)
                  : 0;
            stepSuccPerNode[i] = nRecvdTotal - stepSuccPerNode[i];
        }
        if (stepSuccPerNode[N_BSS] > 1)
        {
            // Get the access delay of VR node
            long double totalAccessDelay = 0.0;
            auto numPkts = stepSuccPerNode[N_BSS] - 1;
            auto nodeDevLinkTuple = std::tuple<uint32_t, uint32_t, uint8_t>(N_BSS, 0, 0);
            const auto& mpduList = successRecords.at(nodeDevLinkTuple);
            auto it = mpduList.end();
            for (auto i = 0; i < numPkts; ++i)
            {
                --it;
                auto prevIt = std::prev(it);
                totalAccessDelay += (it->m_txStartTime - prevIt->m_ackTime).ToDouble(Time::MS);
            }
            vrAccessDelayMs = totalAccessDelay / numPkts;
        }
    }

    // 1. Observation of RX power in BSS0
    // To store RX power matrix in map:
    // id = (RX node # in BSS0) << 5 | (TX node id)
    for (auto i = 0; i < wifiNodes.GetN(); ++i) // TX node id = i
    {
        for (auto j = 0; j < wifiNodes.GetN(); ++j) // RX node id = j
        {
            if (i == j || bssOfNode[j] != 0)
            {
                continue;
            }
            auto indexInBss0 = j / N_BSS;
            uint8_t measId = (static_cast<uint8_t>(indexInBss0) << 5) |
                (static_cast<uint8_t>(i) & 0x1f);
            auto meas = CreateObject<NetworkStats>("MultiBss", measId,
                Simulator::Now().GetMilliSeconds());
            meas->Append("Cpp2Py::RxPowerDbmMatrix", nodeRxPower[i][j]);
            dataProcessor->AppendMeasurement(meas);
        }
    }

    // 2. Observation of MCS in BSS0
    for (auto i = 0; i < wifiNodes.GetN(); ++i)
    {
        if (bssOfNode[i] != 0)
        {
            continue;
        }
        // The 'id' is node # in BSS0
        auto meas = CreateObject<NetworkStats>("MultiBss", i / N_BSS,
            Simulator::Now().GetMilliSeconds());
        meas->Append("Cpp2Py::McsIndex", nodeMcs[i]);
        dataProcessor->AppendMeasurement(meas);
    }

    // 3. Observation of uplink throughput of every node
    for (auto i = 0; i < wifiNodes.GetN(); ++i)
    {
        if (i < N_BSS)  // APs
        {
            continue;
        }
        // The 'id' is node ID
        auto meas = CreateObject<NetworkStats>("MultiBss", i,
            Simulator::Now().GetMilliSeconds());
        meas->Append("Cpp2Py::UplinkThptMbps", static_cast<long double>(stepSuccPerNode[i]) * pktSize * 8 / 1000000);
        std::cout << "obs: node " << i << " thpt " << static_cast<long double>(stepSuccPerNode[i]) * pktSize * 8 / 1000000 << std::endl;
        dataProcessor->AppendMeasurement(meas);
    }

    // 4. Observation of access delay of VR node in BSS0 (node ID = N_BSS)
    auto measDelay = CreateObject<NetworkStats>("MultiBss", N_BSS, Simulator::Now().GetMilliSeconds());
    measDelay->Append("Cpp2Py::AccessDelayMs", vrAccessDelayMs);
    dataProcessor->AppendMeasurement(measDelay);

    // 5. (New) observation of nodes' location (x and y) for visualization
    for (auto i = 0; i < wifiNodes.GetN(); ++i)
    {
        auto meas = CreateObject<NetworkStats>("MultiBss", i, Simulator::Now().GetMilliSeconds());
        auto x = DynamicCast<MobilityModel>(wifiNodes.Get(i)->GetObject<MobilityModel>())->GetPosition().x;
        auto y = DynamicCast<MobilityModel>(wifiNodes.Get(i)->GetObject<MobilityModel>())->GetPosition().y;
        meas->Append("Cpp2Py::NodeX", x);
        meas->Append("Cpp2Py::NodeY", y);
        std::cout << "send loc x=" << x << ", y=" << y << std::endl;
        dataProcessor->AppendMeasurement(meas);
    }

    Simulator::Schedule(measInterval, &GenerateMeasurement);
}

void
RecvAction(const json& action)
{
    if (action == nullptr)
    {
        return;
    }
    auto nextCca = action.get<int>();
    std::cout << "at " << Simulator::Now().ToDouble(Time::MS) << " ms, " << "action: CcaNew=" << nextCca << std::endl;
    // Change CCA of nodes in BSS-0
    for (uint32_t i = 0; i < wifiNodes.GetN(); i += N_BSS)
    {
        uint32_t nodeId = wifiNodes.Get(i)->GetId();
        Ptr<NetDevice> dev = wifiNodes.Get(i)->GetDevice(0);
        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
        Ptr<WifiPhy> wifi_phy = wifi_dev->GetPhy();
        Ssid ssid = wifi_dev->GetMac()->GetSsid();
        NS_ASSERT(ssid.IsEqual(Ssid("BSS-0")));
        double currentCca = wifi_phy->GetCcaSensitivityThreshold();
        Ptr<ThresholdPreambleDetectionModel> preambleCaptureModel =
            CreateObject<ThresholdPreambleDetectionModel>();
        preambleCaptureModel->SetAttribute("MinimumRssi", DoubleValue(nextCca));
        wifi_phy->SetCcaSensitivityThreshold(nextCca);
        wifi_phy->SetPreambleDetectionModel(preambleCaptureModel);
        std::cout << "-- " << ssid << " Node " << nodeId << " current CCA " << currentCca
            << " next CCA " << nextCca << std::endl;
    }
}

std::map<int, std::string>
readConfigFile(const std::string& filename)
{
    std::map<int, std::string> config;

    std::ifstream configFile(filename);
    if (!configFile)
    {
        std::cerr << "Error opening configuration file: " << filename << std::endl;
        return config;
    }

    std::string line;
    std::string ready;
    while (std::getline(configFile, line))
    {
        if (!line.find('#'))
        {
            continue;
        }
        size_t delimiterPos = line.find(':');
        if (delimiterPos != std::string::npos)
        {
            // std::cout << line.substr(0, delimiterPos) << std::endl;
            int node = std::stoi(line.substr(0, delimiterPos));
            std::string configLine = line.substr(delimiterPos + 1);
            config[node] = configLine;
        }
    }

    configFile.close();
    return config;
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
    dataProcessor->SetNetworkGymActionCallback("MultiBss::Py2Cpp::CcaNew", 0, MakeCallback(&RecvAction));

    bool pcap = false; ///< Flag to enable/disable PCAP files generation
    uint32_t seedNumber = 2;

    double frequency = 5; ///< The operating frequency band in GHz: 2.4, 5 or 6
    uint16_t channelWidths = 20; ///< The constant channel width in MHz (only for 11n/ac/ax)
    uint16_t guardIntervalNs = 800;
    ///< The guard interval in nanoseconds (800 or 400 for
                                        ///< 11n/ac, 800 or 1600 or 3200 for 11 ax)
    uint16_t pktInterval =
        5000;
    ///< The socket packet interval in microseconds (a higher value is needed to reach
       ///< saturation conditions as the channel bandwidth or the MCS increases)

    txPower = 16;
    ///< The transmit power of all nodes in dBm (or if --app=setup-done, custom
                     ///< txPowers)

    networkSize = 4;

    Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
                       StringValue("22000"));
    Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("22000"));
    // Disable short retransmission failure (make retransmissions persistent)
    Config::SetDefault("ns3::WifiMac::FrameRetryLimit",
                       UintegerValue(65535));
    // Set maximum queue size to the largest value and set maximum queue delay to be larger than
    // the simulation time
    Config::SetDefault("ns3::WifiMacQueue::MaxSize",
                       QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS,
                                                100)));
    Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(Seconds(20 * stopTime.ToDouble(Time::S))));

    // std::string configFileName = "../../scratch/config.txt";
    std::string configFileName = "../scratch/config.txt";
    CommandLine cmd(__FILE__);
    cmd.AddValue("pktSize", "The packet size in bytes", pktSize);
    cmd.AddValue("rng", "The seed run number", seedNumber);
    // cmd.AddValue("app",
    //              "The type of application to set. (constant,bursty,bursty-trace,setup,setup-done)",
    //              appType);   // setup-done
    cmd.AddValue("prop", "The propagation loss model", propagationModel);
    // cmd.AddValue("ring", "Set ring topology or not", ring);
    cmd.AddValue("pcap", "Enable/disable PCAP tracing", pcap);
    // cmd.AddValue("traceFolder", "The folder containing the trace.", traceFolder);
    // cmd.AddValue("traceFile", "The trace file name.", traceFile);
    cmd.AddValue("networkSize", "Number of stations per bss", networkSize);
    cmd.AddValue("apNodes", "Number of APs", apNodeCount); // use 4
    cmd.AddValue("frequency", "Set the operating frequency band in GHz: 2.4, 5 or 6", frequency);
    cmd.AddValue("channelWidth",
                 "Set the constant channel width in MHz (only for 11n/ac/ax)",
                 channelWidths);
    cmd.AddValue("gi",
                 "Set the the guard interval in nanoseconds (800 or 400 for 11n/ac, 800 or 1600 or "
                 "3200 for 11 ax)",
                 guardIntervalNs);
    cmd.AddValue("maxMpdus",
                 "Set the maximum number of MPDUs in A-MPDUs (0 to disable MPDU aggregation)",
                 maxMpdus);
    cmd.AddValue("distance", "Set the distance in meters between the AP and the STAs", distance);
    cmd.AddValue("txPower", "Set the transmit power of all nodes in dBm", txPower);
    cmd.AddValue("pktInterval", "Set the socket packet interval in microseconds", pktInterval);
    cmd.AddValue("boxsize", "Set the size of the box in meters", boxSize);
    cmd.AddValue("configFile", "Configuration file of Multi-BSS example", configFileName);
    cmd.Parse(argc, argv);

    RngSeedManager::SetSeed(seedNumber);
    RngSeedManager::SetRun(seedNumber);

    for (int i = 0; i < 12; i++)
    {
        dataRateToMcs[HePhy::GetDataRate(i, channelWidths, NanoSeconds(guardIntervalNs), i / 8 + 1)]
            = i;
    }

    int gi = guardIntervalNs;
    apNodes.Create(apNodeCount);
    staNodes.Create(apNodeCount * networkSize);
    // setup-done
    configuration = readConfigFile(configFileName);

    WifiStandard wifiStandard = WIFI_STANDARD_80211ax;

    YansWifiChannelHelper wifiChannel;

    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    if (frequency == 6)
    {
        if (propagationModel == "log")
        {
            // Reference Loss for Friss at 1 m with 6.0 GHz
            wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                           "Exponent",
                                           DoubleValue(2.0),
                                           "ReferenceDistance",
                                           DoubleValue(1.0),
                                           "ReferenceLoss",
                                           DoubleValue(49.013));
        }
        else if (propagationModel == "tgax")
        {
            wifiChannel.AddPropagationLoss("ns3::TgaxResidentialPropagationLossModel",
                                           "Frequency",
                                           DoubleValue(6e9),
                                           "ShadowSigma",
                                           DoubleValue(5.0));
        }
        else if (propagationModel == "fixed")
        {
            wifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(-71));
        }
    }
    else if (frequency == 5)
    {
        // Reference Loss for Friss at 1 m with 5.15 GHz
        if (propagationModel == "log")
        {
            wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                           "Exponent",
                                           DoubleValue(3.0),
                                           "ReferenceDistance",
                                           DoubleValue(1.0),
                                           "ReferenceLoss",
                                           DoubleValue(50));
        }
        else if (propagationModel == "tgax")
        {
            wifiChannel.AddPropagationLoss("ns3::TgaxResidentialPropagationLossModel",
                                           "Frequency",
                                           DoubleValue(5e9),
                                           "ShadowSigma",
                                           DoubleValue(5.0));
        }
        else if (propagationModel == "fixed")
        {
            wifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(-71));
        }
    }
    else
    {
        // Reference Loss for Friss at 1 m with 2.4 GHz
        if (propagationModel == "log")
        {
            wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                           "Exponent",
                                           DoubleValue(2.0),
                                           "ReferenceDistance",
                                           DoubleValue(1.0),
                                           "ReferenceLoss",
                                           DoubleValue(40.046));
        }
        else if (propagationModel == "tgax")
        {
            wifiChannel.AddPropagationLoss("ns3::TgaxResidentialPropagationLossModel",
                                           "Frequency",
                                           DoubleValue(2.4e9),
                                           "ShadowSigma",
                                           DoubleValue(5.0));
        }
        else if (propagationModel == "fixed")
        {
            wifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(-71));
        }
    }

    WifiHelper wifi;
    wifi.SetStandard(wifiStandard);
    Config::SetDefault("ns3::AutoMcsWifiManager::autoMCS", BooleanValue(true));
    wifi.SetRemoteStationManager("ns3::AutoMcsWifiManager");
    YansWifiPhyHelper phy;
    phy.SetErrorRateModel("ns3::NistErrorRateModel");

    phy.SetChannel(wifiChannel.Create());
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    uint64_t beaconInterval = 100 * 1024;

    WifiMacHelper mac;
    for (int i = 0; i < apNodeCount; ++i)
    {
        std::string configString = configuration[apNodes.Get(i)->GetId()];
        std::vector<std::string> configValues = csv_split(configString, ',');
        double m_ccaSensitivity = std::stoi(configValues[1]);
        double m_txPower = std::stoi(configValues[2]);

        phy.Set("CcaSensitivity", DoubleValue(m_ccaSensitivity));
        phy.SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel",
                                      "MinimumRssi",
                                      DoubleValue(m_ccaSensitivity));
        phy.Set("TxPowerStart", DoubleValue(m_txPower));
        phy.Set("TxPowerEnd", DoubleValue(m_txPower));
        std::string chStr = "{" + configValues[4] + "," + configValues[3] + ", BAND_5GHZ, 0}";
        phy.Set("ChannelSettings", StringValue(chStr));
        std::string ssi = "BSS-" + std::to_string(i);
        Ssid ssid = Ssid(ssi);
        bssOfNode[apNodes.Get(i)->GetId()] = i;
        mac.SetType("ns3::ApWifiMac",
                    "BeaconInterval",
                    TimeValue(MicroSeconds(beaconInterval)),
                    "Ssid",
                    SsidValue(ssid));
        NetDeviceContainer tmp = wifi.Install(phy, mac, apNodes.Get(i));

        apDevices.Add(tmp.Get(0));
        devices.Add(tmp.Get(0));
        wifiNodes.Add(apNodes.Get(i));
        std::cout << "AP MAC: " << tmp.Get(0)->GetAddress() << "," << ssi << std::endl;
    }
    phy.EnablePcap("AP", apDevices);

    for (uint32_t i = 0; i < (apNodeCount * networkSize); ++i)
    {
        std::string configString = configuration[staNodes.Get(i)->GetId()];

        std::vector<std::string> configValues = csv_split(configString, ',');

        std::cout << "STA node id " << staNodes.Get(i)->GetId() << " : " << configValues[0]
            << ", " << configValues[1] << ", " << configValues[2] << ", "
            << configValues[3] << ", " << configValues[4] << ", " << std::endl;

        double m_ccaSensitivity = std::stoi(configValues[1]);
        double m_txPower = std::stoi(configValues[2]);

        phy.Set("CcaSensitivity", DoubleValue(m_ccaSensitivity));
        phy.SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel",
                                      "MinimumRssi",
                                      DoubleValue(m_ccaSensitivity));
        phy.Set("TxPowerStart", DoubleValue(m_txPower));
        phy.Set("TxPowerEnd", DoubleValue(m_txPower));

        std::string chStr = "{" + configValues[4] + "," + configValues[3] + ", BAND_5GHZ, 0}";
        phy.Set("ChannelSettings", StringValue(chStr));

        std::string ssi = "BSS-" + std::to_string(i % apNodeCount);
        Ssid ssid = Ssid(ssi);
        bssOfNode[staNodes.Get(i)->GetId()] = i % apNodeCount;
        mac.SetType("ns3::StaWifiMac",
                    "MaxMissedBeacons",
                    UintegerValue(std::numeric_limits<uint32_t>::max()),
                    "Ssid",
                    SsidValue(ssid));
        NetDeviceContainer tmp = wifi.Install(phy, mac, staNodes.Get(i));

        devices.Add(tmp.Get(0));
        staDevices.Add(tmp.Get(0));
        wifiNodes.Add(staNodes.Get(i));

        std::cout << "STA: " << i << std::endl;
        std::cout << "STA MAC: " << tmp.Get(0)->GetAddress() << "," << ssi << std::endl;
    }

    WifiHelper::AssignStreams(devices, 0);

    // Set guard interval
    Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/"
                "GuardInterval",
                TimeValue(NanoSeconds(gi)));

    // Configure AP aggregation
    for (int i = 0; i < apNodeCount; ++i)
    {
        Ptr<NetDevice> dev = apNodes.Get(i)->GetDevice(0);

        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
        wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (pktSize + 50)));
        wifi_dev->GetMac()->SetAttribute("BK_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (pktSize + 50)));
        wifi_dev->GetMac()->SetAttribute("VO_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (pktSize + 50)));
        wifi_dev->GetMac()->SetAttribute("VI_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (pktSize + 50)));
    }
    // Configure STA aggregation
    for (uint32_t i = 0; i < (apNodeCount * networkSize); ++i)
    {
        Ptr<NetDevice> dev = staNodes.Get(i)->GetDevice(0);

        Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
        wifi_dev->GetMac()->SetAttribute("BE_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (pktSize + 50)));
        wifi_dev->GetMac()->SetAttribute("BK_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (pktSize + 50)));
        wifi_dev->GetMac()->SetAttribute("VO_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (pktSize + 50)));
        wifi_dev->GetMac()->SetAttribute("VI_MaxAmpduSize",
                                         UintegerValue(maxMpdus * (pktSize + 50)));
    }

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // create a set of rooms in a building

    double xRoomCount = apNodeCount;
    double yRoomCount = 1;
    if (apNodeCount >= 3)
    {
        xRoomCount = 2;
        yRoomCount = 2;
    }

    double floorCount = 1;

    double buildingSizeX = boxSize * xRoomCount; // m
    double buildingSizeY = boxSize * yRoomCount; // m
    double buildingHeight = 3 * floorCount; // m

    Ptr<Building> building;
    building = CreateObject<Building>();

    building->SetBoundaries(Box(0, buildingSizeX, 0, buildingSizeY, 0, buildingHeight));
    building->SetNRoomsX(xRoomCount);
    building->SetNRoomsY(yRoomCount);
    building->SetNFloors(floorCount);

    randomX->SetAttribute("Stream", IntegerValue(seedNumber));
    randomX->SetAttribute("Max", DoubleValue(boxSize));
    randomX->SetAttribute("Min", DoubleValue(0.0));

    randomY->SetAttribute("Stream", IntegerValue(seedNumber + 1));
    randomY->SetAttribute("Max", DoubleValue(boxSize));
    randomY->SetAttribute("Min", DoubleValue(0.0));

    for (uint32_t i = 0; i < apNodes.GetN(); i++)
    {
        double x = randomX->GetValue();
        double y = randomY->GetValue();
        if (i == 1)
        {
            x = (boxSize / 2) + (boxSize);
            y = (boxSize / 2);
        }
        if (i == 2)
        {
            x = (boxSize / 2);
            y = (boxSize / 2) + (boxSize);
        }
        else if (i == 3)
        {
            x = (boxSize / 2) + (boxSize);
            y = (boxSize / 2) + (boxSize);
        }
        Vector l1(x, y, 1.5);
        positionAlloc->Add(l1);
        std::cout << "AP" << i << " " << x << "," << y << std::endl;
    }
    std::vector<Vector> ringPos;
    // Set postion for STAs
    for (uint32_t i = 0; i < staNodes.GetN(); i++)
    {
        double x = randomX->GetValue();
        double y = randomY->GetValue();
        double currentAp = bssOfNode[staNodes.Get(i)->GetId()];
        if (currentAp == 1)
        {
            x = x + (boxSize);
        }
        if (currentAp == 2)
        {
            y = y + (boxSize);
        }
        else if (currentAp == 3)
        {
            x = x + (boxSize);
            y = y + (boxSize);
        }
        Vector l1(x, y, 1.5);
        positionAlloc->Add(l1);
        std::cout << "STA" << i << " " << x << "," << y << std::endl;
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(wifiNodes);
    BuildingsHelper::Install(wifiNodes);

    Ptr<UniformRandomVariable> startTime = CreateObject<UniformRandomVariable>();
    startTime->SetAttribute("Stream", IntegerValue(0));
    startTime->SetAttribute("Min", DoubleValue(6));
    startTime->SetAttribute("Max", DoubleValue(8));

    ApplicationContainer apps;

    InternetStackHelper stack;
    stack.Install(wifiNodes);

    uint16_t portNumber = 50000;

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");

    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);
    Ipv4InterfaceContainer apInterfaces = address.Assign(apDevices);

    for (int i = 0; i < apNodeCount; i++)
    {
        Ipv4Address apAddress = apInterfaces.GetAddress(i);
        // Create bursty application helper
        BurstyHelper burstyHelper("ns3::UdpSocketFactory",
                                  InetSocketAddress(apAddress, portNumber));
        burstyHelper.SetAttribute("FragmentSize", UintegerValue(pktSize));

        // Create burst sink helper
        BurstSinkHelper burstSinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(apAddress, portNumber));

        // Install burst sink
        ApplicationContainer apApps = burstSinkHelper.Install(apNodes.Get(i));
        Ptr<BurstSink> burstSink = apApps.Get(0)->GetObject<BurstSink>();

        for (uint32_t x = 0; x < staNodes.GetN(); x += apNodeCount)
        {
            std::string trafficType = configuration[staNodes.Get(x + i)->GetId()];
            size_t pos = trafficType.find(',');
            trafficType = trafficType.substr(0, pos);
            std::cout << "Sta: " << staNodes.Get(x + i)->GetId() << " Traffic " << trafficType
                << std::endl;
            if (trafficType == "constant")
            {
                Ptr<WifiNetDevice> wifi_apDev = DynamicCast<WifiNetDevice>(apDevices.Get(i));
                Ptr<ApWifiMac> ap_mac = DynamicCast<ApWifiMac>(wifi_apDev->GetMac());
                Ptr<PacketSocketServer> server = CreateObject<PacketSocketServer>();
                Ptr<WifiNetDevice> wifi_staDev =
                    DynamicCast<WifiNetDevice>(staDevices.Get(x + i));
                Ptr<StaWifiMac> sta_mac = DynamicCast<StaWifiMac>(wifi_staDev->GetMac());

                PacketSocketAddress socketAddr;
                socketAddr.SetSingleDevice(staDevices.Get((x + i))->GetIfIndex());
                socketAddr.SetPhysicalAddress(apDevices.Get(i)->GetAddress());
                socketAddr.SetProtocol(1);

                Ptr<PacketSocketClient> client = CreateObject<PacketSocketClient>();
                client->SetRemote(socketAddr);

                staNodes.Get(x + i)->AddApplication(client);
                client->SetAttribute("PacketSize", UintegerValue(pktSize));
                client->SetAttribute("MaxPackets", UintegerValue(0));
                client->SetAttribute("Interval", TimeValue(Time(MicroSeconds(pktInterval))));
                client->SetStartTime(Seconds(startTime->GetValue()));

                server->SetLocal(socketAddr);
                if (x == 0)
                {
                    apNodes.Get(i)->AddApplication(server);
                }
            }
            else if (trafficType == "bursty")
            {
                burstyHelper.SetBurstGenerator(
                    "ns3::SimpleBurstGenerator",
                    "PeriodRv",
                    StringValue("ns3::ConstantRandomVariable[Constant=5e-3]"),
                    "BurstSizeRv",
                    StringValue("ns3::ConstantRandomVariable[Constant=25e3]"));

                // Install bursty application
                ApplicationContainer staApps = burstyHelper.Install(staNodes.Get(i + x));
                Ptr<BurstyApplication> burstyApp =
                    staApps.Get(0)->GetObject<BurstyApplication>();
            }
        }
    }

    // TX stats
    wifiTxStats.Enable(devices);
    wifiTxStats.Start(Seconds(1));
    wifiTxStats.Stop(stopTime + Seconds(1));

    Simulator::Stop(stopTime + Seconds(1));
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
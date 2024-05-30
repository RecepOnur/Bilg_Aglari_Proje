#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/trace-helper.h"
#include "ns3/config.h"
#include "ns3/network-module.h"
#include "ns3/network-server-helper.h"
#include "ns3/forwarder-helper.h"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("SimpleLorawanNetworkWithMobilityExample");

uint32_t packetsSent = 0;
uint32_t packetsReceived = 0;
double totalDelay = 0.0;

void PacketSentCallback (Ptr<const Packet> packet)
{
    packetsSent++;
    NS_LOG_INFO("Packet sent at " << Simulator::Now().GetSeconds() << " seconds");
}

void PacketReceivedCallback (Ptr<const Packet> packet, const Address &address)
{
    packetsReceived++;
    double delay = Simulator::Now().GetSeconds() - packet->GetUid(); 
    totalDelay += delay;
    NS_LOG_INFO("Packet received at " << Simulator::Now().GetSeconds() << " seconds with delay " << delay);
}

int main(int argc, char* argv[]) {
    // Log ayarları
    LogComponentEnable("SimpleLorawanNetworkWithMobilityExample", LOG_LEVEL_ALL);
    LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
    LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
    LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
    LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
    LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
    LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
    LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
    LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
    LogComponentEnable("OneShotSenderHelper", LOG_LEVEL_ALL);
    LogComponentEnable("OneShotSender", LOG_LEVEL_ALL);
    LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
    LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnableAll(LOG_PREFIX_NODE);
    LogComponentEnableAll(LOG_PREFIX_TIME);

    // Komut satırı parametreleri
    std::string traceFile = "/home/onur/ns-allinone-3.41/ns-3.41/scratch/ns2mobility.tcl";
    CommandLine cmd;
    cmd.AddValue("traceFile", "NS2 movement trace file", traceFile);
    cmd.Parse(argc, argv);

    // Trace file check
    std::ifstream file(traceFile.c_str());
    if (!file.is_open()) {
        NS_LOG_UNCOND("Could not open trace file " << traceFile << " for reading, aborting here");
        return 1;
    } else {
        NS_LOG_UNCOND("Trace file opened successfully!");
        file.close();
    }

    /************************
     *  Kanal oluşturma     *
     ************************/
    NS_LOG_INFO("Creating the channel...");

    // Lora kanal nesnesini oluştur
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();

    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    /************************
     *  Yardımcıları oluştur *
     ************************/
    NS_LOG_INFO("Setting up helpers...");

    // Mobilite yardımcısını oluştur ve iz dosyasından mobilite modelini ayarla
    MobilityHelper mobility;
    Ns2MobilityHelper ns2 = Ns2MobilityHelper(traceFile);

   
     

    // LoraPhyHelper'ı oluştur
    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    // LorawanMacHelper'ı oluştur
    LorawanMacHelper macHelper = LorawanMacHelper();
    macHelper.SetRegion(LorawanMacHelper::EU);  // Avrupa bölgesi ayarlandı

    // LoraHelper'ı oluştur
    LoraHelper helper = LoraHelper();

    /************************
     *  Son Cihazları Oluştur*
     ************************/
    NS_LOG_INFO("Creating the end device...");

    // Bir dizi düğüm oluştur
    NodeContainer endDevices;
    endDevices.Create(2);
    

    // İz dosyasından düğümlere mobilite modeli ata
    ns2.Install();  // Install mobility model to all nodes

    uint32_t numDevices = endDevices.GetN();
    if (numDevices != 2) {
        NS_LOG_ERROR("Failed to create two end devices!");
    }

    // Son cihazların LoraNetDevices'ını oluştur
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    helper.Install(phyHelper, macHelper, endDevices);

    // Paket gönderimi ve alımını izlemek için bağlantı yap
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::lorawan::EndDeviceLorawanMac/Tx", MakeCallback(&PacketSentCallback));
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::lorawan::GatewayLorawanMac/Rx", MakeCallback(&PacketReceivedCallback));

    /*********************
     *  Ağ Geçitlerini Oluştur *
     *********************/
    NS_LOG_INFO("Creating the gateway...");
    NodeContainer gateways;
    gateways.Create(1);

    // Ağ geçidini sabit bir konuma ayarla
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(0.0, 0.0, 0.0));
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gateways);

    // Her ağ geçidi için bir ağ cihazı oluştur
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    /*********************************************
     *  Son cihazlara uygulamalar kur *
     *********************************************/
    OneShotSenderHelper oneShotSenderHelper;
    oneShotSenderHelper.SetSendTime(Seconds(2));
    oneShotSenderHelper.Install(endDevices);

    /******************
     * Veri hızlarını ayarla *
     ******************/
    std::vector<int> sfQuantity(6);
    sfQuantity = LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    Ptr<Node> networkServer = CreateObject<Node>();

    // PointToPoint links between gateways and server
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));
    // Store network server app registration details for later
    P2PGwRegistration_t gwRegistration;
    for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
    {
        auto container = p2p.Install(networkServer, *gw);
        auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
        gwRegistration.emplace_back(serverP2PNetDev, *gw);
    }

    // Install the NetworkServer application on the network server
    NetworkServerHelper networkServerHelper;
    networkServerHelper.SetGatewaysP2P(gwRegistration);
    networkServerHelper.SetEndDevices(endDevices);
    networkServerHelper.Install(networkServer);

    // Install the Forwarder application on the gateways
    ForwarderHelper forwarderHelper;
    forwarderHelper.Install(gateways);

    /****************
     *  Simülasyon  *
     ****************/
    Simulator::Stop(Seconds(800));
    Simulator::Run();
    Simulator::Destroy();

    // PDR ve ortalama gecikmeyi hesapla ve yazdır
    double pdr = static_cast<double>(packetsReceived) / packetsSent;
    double avgDelay = packetsReceived > 0 ? totalDelay / packetsReceived : 0.0;
    std::cout << "Packet Delivery Rate: " << pdr << std::endl;
    std::cout << "Average Delay: " << avgDelay << " seconds" << std::endl;

    return 0;
}

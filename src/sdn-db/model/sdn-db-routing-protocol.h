/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Haoliang Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Haoliang Chen <chl41993@gmail.com>
 */

#ifndef SDN_DB_IMPL_H
#define SDN_DB_IMPL_H

#include "sdn-db-header.h"

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/socket.h"
#include "ns3/event-garbage-collector.h"
#include "ns3/random-variable-stream.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/mobility-module.h"
//#include "sdn-duplicate-detection.h"

#include <vector>
#include <map>


namespace ns3 {
namespace sdndb {


enum NodeType {CAR, LOCAL_CONTROLLER, GLOBAL_CONTROLLER, OTHERS};

//车辆的方向，start to end 和 end to start两种
enum CarDirect {S2E, E2S};

/// An SDN's routing table entry.
struct RoutingTableEntry
{
  RoutingTableEntry () : // default values
                           destAddr (uint32_t(0)),
                           nextHop (uint32_t(0)),
                           mask (uint32_t(0)),
                           interface (0) {};

  Ipv4Address destAddr; ///< Address of the destination subnet.
  Ipv4Address nextHop; ///< Address of the next hop.
  Ipv4Address mask; ///< mask of the destination subnet.
  uint32_t interface; ///< Interface index.
};

// A struct for LC to hold Information that got from cars
class CarInfo
{
public:

  CarInfo () :
    Active (false)
  {
	    distostart = INFINITY;
	    direct = S2E;
  };

  //Get position by this time
  Vector3D GetPos () const
  {
    double t = Simulator::Now ().GetSeconds () - LastActive.GetSeconds ();
    double x = this->Position.x + this->Velocity.x * t,
           y = this->Position.y + this->Velocity.y * t,
           z = this->Position.z + this->Velocity.z * t;
    return Vector3D(x, y, z);
  }
  Vector3D Position;
  Vector3D Velocity;
  Time LastActive;//Timeout indicator
  bool Active;
  std::vector<RoutingTableEntry> R_Table;
  double distostart;
  CarDirect direct;
};

//LC的个数+1,不使用0
#define LC_NUM 25
class LcGraph
{
private:
	int S2E[LC_NUM];//记录每个lc正方向链接的跳数【后期可以考虑扩展为每个lc正方向链接的数目】
	int E2S[LC_NUM];//记录每个lc反方向链接的跳数
	int w[LC_NUM][LC_NUM];//边的权重，即两条路的跳数之和
	int d[LC_NUM][LC_NUM];//从一个lc到另一个lc需要走哪个方向，0表示两个都是反方向，1是反正，2是正反，3是正正
public:
	LcGraph();
	void InsertW(int i, int j, int sum);
	void SetS2E(int i, int num);
	void SetE2S(int i, int num);
	void BuildGraph();
	std::vector<int> Floyd(int s, int d);
};
struct ShortHop
{
  ShortHop ()
  {
    hopnumber = INFINITY;
    isTransfer = false;
    nextID = Ipv4Address::GetZero ();
    IDa = Ipv4Address::GetZero ();
    IDb = Ipv4Address::GetZero ();
    proxyID = Ipv4Address::GetZero ();
    t = 0;
  };

  Ipv4Address nextID;
  uint32_t hopnumber;
  bool isTransfer;
  Ipv4Address IDa, IDb, proxyID;
  double t; //in secends
};

class RoutingProtocol;

/// \brief SDN routing protocol for IPv4
///
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static TypeId GetTypeId (void);//implemented

  RoutingProtocol ();//implemented
  virtual ~RoutingProtocol ();//implemented

  ///
  /// \brief Set the SDN main address to the first address on the indicated
  ///        interface
  /// \param interface IPv4 interface index
  ///
  void SetSCHInterface (uint32_t interface);//implemented
  void SetCCHInterface (uint32_t interface);//implemented

  ///
  /// Dump the routing table
  /// to logging output (NS_LOG_DEBUG log level).  If logging is disabled,
  /// this function does nothing.
  ///
  void Dump (void);//implemented

  /**
   * Return the list of routing table entries discovered by SDN
   **/
  std::vector<RoutingTableEntry> GetRoutingTableEntries () const;//implemented

 /**
  * Assign a fixed random variable stream number to the random variables
  * used by this model.  Return the number of streams (possibly zero) that
  * have been assigned.
  *
  * \param stream first stream index to use
  * \return the number of stream indices assigned by this model
  */
  int64_t AssignStreams (int64_t stream);//implemented

private:
  std::set<uint32_t> m_interfaceExclusions;

public:
  std::set<uint32_t> GetInterfaceExclusions () const
  {
    return (m_interfaceExclusions);
  }
  void SetInterfaceExclusions (std::set<uint32_t> exceptions);//implemented

protected:
  virtual void DoInitialize (void);//implemented
private:
  std::map<Ipv4Address, RoutingTableEntry> m_table; ///< Data structure for the routing table. (Use By Mainly by CAR Node, but LC needs it too)

  std::map<Ipv4Address, CarInfo> m_lc_info;///for LC
  std::map<Ipv4Address, CarInfo> m_lc_infoS;///for LC to store cars with S2E direction
  std::map<Ipv4Address, CarInfo> m_lc_infoE;///for LC to store cars with E2S direction

  EventGarbageCollector m_events;
	
  /// Packets sequence number counter.
  uint16_t m_packetSequenceNumber;
  /// Messages sequence number counter.
  uint16_t m_messageSequenceNumber;

  /// HELLO messages' emission interval.
  Time m_helloInterval;
  /// Routing messages' emission interval.
  Time m_rmInterval;
  /// minimum ap message emission interval
  Time m_minAPInterval;

  Ptr<Ipv4> m_ipv4;

  void Clear ();//implemented
  uint32_t GetSize () const { return (m_table.size ()); }
  void RemoveEntry (const Ipv4Address &dest);//implemented
  //用来更新RoutingTableEntry
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &mask,
                 const Ipv4Address &next,
                 uint32_t interface);//implemented
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &mask,
                 const Ipv4Address &next,
                 const Ipv4Address &interfaceAddress);//implemented
  bool Lookup (const Ipv4Address &dest,
               RoutingTableEntry &outEntry) const;//implemented

  // From Ipv4RoutingProtocol
  //把自身携带的包转出去，优先转发
  virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p,
                                      const Ipv4Header &header,
                                      Ptr<NetDevice> oif,
                                      Socket::SocketErrno &sockerr);//implemented
  //收到包之后决定是否需要转发
  virtual bool RouteInput (Ptr<const Packet> p,
                           const Ipv4Header &header,
                           Ptr<const NetDevice> idev,
                           UnicastForwardCallback ucb,
                           MulticastForwardCallback mcb,
                           LocalDeliverCallback lcb,
                           ErrorCallback ecb);//implemented
  virtual void NotifyInterfaceUp (uint32_t interface);//implemented
  virtual void NotifyInterfaceDown (uint32_t interface);//implemented
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);//implemented
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);//implemented
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);//implemented
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const;//implemented

  void DoDispose ();//implemented

  void SendPacket (Ptr<Packet> packet, const MessageList &containedMessages);//implemented

  /// Increments packet sequence number and returns the new value.
  inline uint16_t GetPacketSequenceNumber ();//implemented
  /// Increments message sequence number and returns the new value.
  inline uint16_t GetMessageSequenceNumber ();//implemented

  void RecvSDN (Ptr<Socket> socket);//implemented

  //Ipv4Address GetMainAddress (Ipv4Address iface_addr) const;

  // Timer handlers
  Timer m_helloTimer;
  void HelloTimerExpire ();//implemented

  Timer m_rmTimer;
  void RmTimerExpire ();//implemented

  Timer m_apTimer;
  void APTimerExpire ();

  /// A list of pending messages which are buffered awaiting for being sent.
  sdndb::MessageList m_queuedMessages;
  Timer m_queuedMessagesTimer; // timer for throttling outgoing messages

  void QueueMessage (const sdndb::MessageHeader &message, Time delay);//implemented
  void SendQueuedMessages ();//implemented
  void SendHello ();//implemented
  void SendRoutingMessage (); //Fullfilled
  void SendAppointment();
  void SendCRREQ(const Ipv4Address &destAddress);
  void SendCRREP(const Ipv4Address &sourceAddress,const Ipv4Address &destAddress,const Ipv4Address &transferAddress);
  void ProcessAppointment (const sdndb::MessageHeader &msg);
  void ProcessRm (const sdndb::MessageHeader &msg);//implemented
  void ProcessHM (const sdndb::MessageHeader &msg,const Ipv4Address &senderIface); //implemented
  void ProcessCRREQ (const sdndb::MessageHeader &msg);
  void ProcessCRREP (const sdndb::MessageHeader &msg);
  /*add by xjl 2017-3-1*/
  void SendLclinkMessage (uint32_t s, uint32_t e);
  void SendLcRoutingMessage(std::vector<Ipv4Address> lcresult);
  void ProcessLM(const sdndb::MessageHeader &msg);
  void ComputeRoute ();//a basic version based on distance
  void ComputeRoute2 ();
  void ComputeLcRoute(Ipv4Address sourcelc, Ipv4Address destlc);
  /*end add*/

  /// Check that address is one of my interfaces
  bool IsMyOwnAddress (const Ipv4Address & a) const;//implemented

private:
  Ipv4Address m_SCHmainAddress;
  Ipv4Address m_CCHmainAddress;
  uint32_t m_SCHinterface;
  uint32_t m_CCHinterface;
  std::map<Ipv4Address, Ipv4Address> m_SCHaddr2CCHaddr;
  Ipv4Address transferAddress;//now it is the nearest ip
  Ipv4Address roadendAddress;
  //std::map<Ipv4Address, Ipv4Address> m_SCHaddr2IfaceAddr;
  // One socket per interface, each bound to that interface's address
  // (reason: for VANET-SDN we need to distinguish CCH and SCH interfaces)
  std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_socketAddresses;

  TracedCallback <const PacketHeader &,
                  const MessageList &> m_rxPacketTrace;
  TracedCallback <const PacketHeader &,
                  const MessageList &> m_txPacketTrace;
  TracedCallback <uint32_t> m_routingTableChanged;

  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;  

  // Mobility module for Vanet
  Ptr<MobilityModel> m_mobility;

public:
  void SetMobility (Ptr<MobilityModel> mobility);//implemented

private:
  NodeType m_nodetype;
  //Only node type CAR use this(below)
  AppointmentType m_appointmentResult;
  Ipv4Address m_next_forwarder;

public:
  void SetType (NodeType nt); //implemented
  NodeType GetType () const; //implemented

private:
  bool m_linkEstablished;
  std::vector< std::set<Ipv4Address> > m_Sections;
  ShortHop GetShortHop (const Ipv4Address& IDa, const Ipv4Address& IDb);
  //用来更新RoutingTableEntry中的R_Table
  void LCAddEntry( const Ipv4Address& ID,
                   const Ipv4Address& dest,
                   const Ipv4Address& mask,
                   const Ipv4Address& next);
  void LCAddEntry (const Ipv4Address& ID,
                   const Ipv4Address &dest,
                   const Ipv4Address &mask,
                   const Ipv4Address &next,
                   const Ipv4Address &interfaceAddress);
  void LCAddEntry (const Ipv4Address& ID,
                   const Ipv4Address &dest,
                   const Ipv4Address &mask,
                   const Ipv4Address &next,
                   uint32_t interface);
  void ClearAllTables ();

  int GetArea (Vector3D position) const;
  int GetNumArea () const;
  void Init_NumArea();
  int m_numArea;
  bool m_isPadding;
  bool m_numAreaVaild;

  bool isPaddingExist () const;

  void RemoveTimeOut ();

  double m_road_length;
  double m_signal_range;
public:
  void SetSignalRangeNRoadLength (double signal_range, double road_length);

private:
  void Do_Init_Compute ();
  void Do_Update ();
  void Reschedule ();

  void Partition ();
  void SetN_Init ();
  void OtherSet_Init ();
  void SelectNode ();

  void SortByDistance (int area);
  void CalcShortHopOfArea (int fromArea, int toArea);
  void CalcIntraArea (int area);
  void UpdateMinHop (const Ipv4Address &ID);
  //ResetAppointmentResult In m_lc_info;
  void ResetAppointmentResult ();
  std::list<Ipv4Address> m_list4sort;
  std::map<Ipv4Address, std::list<ShortHop> > m_lc_shorthop;

  void ShiftArea ();
  void AddNewToZero ();
  void CalcSetZero ();
  void SelectNewNodeInAreaZero ();

  Ipv4Address m_theFirstCar;//Use by Reschedule (), SelectNewNodeInAreaZero(); Assign by SelectNode ();
  //Duplicate_Detection m_duplicate_detection;
  //LC放置在道路的中央，保存该LC管辖的范围
  Vector3D m_start;
  Vector3D m_end;
  //配置该道路的两个端点
  void ConfigStartEnd();
  //判断car的方向，S2E指从数值小的一边走到数值大的一边，E2S则相反
  //计算当前车辆距离所在车道起点的距离
  void ConfigDisDirect(Vector3D lastpos,Vector3D currentpos, double &distance, CarDirect &direct);
private:
  LcGraph m_lcgraph;//保存lc地图
  std::map<Ipv4Address,std::map<Ipv4Address, Ipv4Address>> m_gc_info;
  std::map<int, Ipv4Address> m_start_s2e;//保存每个lc正方向链路的第一辆车的ip
  std::map<int, Ipv4Address> m_start_e2s;//保存每个lc反方向链路的第一辆车的ip
  std::vector<Ipv4Address> chosenIp; //这里ip的个数就是S2E边链路的跳数
  std::vector<Ipv4Address> chosenIpe; //这里ip的个数就是E2S边链路的跳数
};


}
}  // namespace ns3

#endif /* SDN_DB_IMPL_H */

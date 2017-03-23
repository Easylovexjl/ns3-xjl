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
 * Author: Haoliang Chen <chl41993@gmail.com>
 */
#include "sdn-db-helper.h"
#include "ns3/node-list.h"
#include "ns3/names.h"
#include "ns3/ptr.h"
#include "ns3/ipv4-list-routing.h"

namespace ns3 {

SdndbHelper::SdndbHelper ()
  : m_rl (814),
    m_sr (419)
{
  m_agentFactory.SetTypeId ("ns3::sdndb::RoutingProtocol");
}

SdndbHelper::SdndbHelper (const SdndbHelper &o)
  : m_agentFactory (o.m_agentFactory),
    m_rl (o.m_rl),
    m_sr (o.m_sr)
{
  m_interfaceExclusions = o.m_interfaceExclusions;
  m_ntmap = o.m_ntmap;
}

SdndbHelper*
SdndbHelper::Copy () const
{
  return new SdndbHelper (*this);
}

void
SdndbHelper::ExcludeInterface (Ptr<Node> node, uint32_t interface)
{
  std::map< Ptr<Node>, std::set<uint32_t> >::iterator it = m_interfaceExclusions.find (node);

  if(it == m_interfaceExclusions.end ())
    {
      std::set<uint32_t> interfaces;
      interfaces.insert (interface);

      m_interfaceExclusions.insert (std::make_pair (node, std::set<uint32_t> (interfaces) ));
    }
  else
    {
      it->second.insert (interface);
    }
}

Ptr<Ipv4RoutingProtocol>
SdndbHelper::Create (Ptr<Node> node) const
{
  Ptr<sdndb::RoutingProtocol> agent = m_agentFactory.Create<sdndb::RoutingProtocol> ();

  std::map<Ptr<Node>, std::set<uint32_t> >::const_iterator it = m_interfaceExclusions.find (node);

  if(it != m_interfaceExclusions.end ())
    {
      agent->SetInterfaceExclusions (it->second);
    }

  Ptr<MobilityModel> temp = node -> GetObject<MobilityModel> ();
  agent->SetMobility (temp);

  std::map< Ptr<Node>, sdndb::NodeType >::const_iterator it3 = m_ntmap.find (node);
  if (it3 != m_ntmap.end ())
    {
      agent->SetType (it3->second);
    }
  else
    {
      agent->SetType (sdndb::OTHERS);
    }
  agent->SetSignalRangeNRoadLength (m_sr, m_rl);


  node->AggregateObject (agent);
  return agent;
}

void
SdndbHelper::Set (std::string name, const AttributeValue &value)
{
  m_agentFactory.Set (name, value);
}

int64_t
SdndbHelper::AssignStreams (NodeContainer c, int64_t stream)
{
  int64_t currentStream = stream;
  Ptr<Node> node;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      node = (*i);
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      NS_ASSERT_MSG (ipv4, "Ipv4 not installed on node");
      Ptr<Ipv4RoutingProtocol> proto = ipv4->GetRoutingProtocol ();
      NS_ASSERT_MSG (proto, "Ipv4 routing not installed on node");
      Ptr<sdndb::RoutingProtocol> sdndb = DynamicCast<sdndb::RoutingProtocol> (proto);
      if (sdndb)
        {
          currentStream += sdndb->AssignStreams (currentStream);
          continue;
        }
      // Sdn may also be in a list
      Ptr<Ipv4ListRouting> list = DynamicCast<Ipv4ListRouting> (proto);
      if (list)
        {
          int16_t priority;
          Ptr<Ipv4RoutingProtocol> listProto;
          Ptr<sdndb::RoutingProtocol> listSdndb;
          for (uint32_t i = 0; i < list->GetNRoutingProtocols (); i++)
            {
              listProto = list->GetRoutingProtocol (i, priority);
              listSdndb = DynamicCast<sdndb::RoutingProtocol> (listProto);
              if (listSdndb)
                {
                  currentStream += listSdndb->AssignStreams (currentStream);
                  break;
                }
            }
        }
    }
  return (currentStream - stream);

}

void
SdndbHelper::SetNodeTypeMap (Ptr<Node> node, sdndb::NodeType nt)
{
  std::map< Ptr<Node> , sdndb::NodeType >::iterator it = m_ntmap.find(node);

  if (it != m_ntmap.end() )
    {
      std::cout<<"Duplicate NodeType on Node: "<< node->GetId()<<std::endl;
    }
  m_ntmap[node] = nt;
}

void
SdndbHelper::SetRLnSR(double signal_range, double road_length)
{
  m_sr = signal_range;
  m_rl = road_length;
}

} // namespace ns3

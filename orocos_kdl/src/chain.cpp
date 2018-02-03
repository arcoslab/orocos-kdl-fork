// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "chain.hpp"
#include <iostream>

namespace KDL {
    using namespace std;

    Chain::Chain():
        segments(0),
        nrOfJoints(0),
    nrOfIndJoints(0),
        nrOfSegments(0),
    cm(),
    nrOfUnlockedJoints(0),
    locked_joints(0)
    {
    }

    Chain::Chain(const Chain& in):nrOfJoints(0),
                                  nrOfSegments(0),nrOfUnlockedJoints(0),
                  nrOfIndJoints(0),segments(0),cm(),
                  locked_joints(0)
    {
      //std::cout<<"copying chain constructor"<<std::endl;
        for(unsigned int i=0;i<in.getNrOfSegments();i++)
            this->addSegment(in.getSegment(i));
    this->setCoupling(in.getLockedJoints(),in.cm);
    }

    Chain& Chain::operator=(const Chain& arg)
    {
      //std::cout<<"copying operator chain"<<std::endl;
        nrOfJoints=0;
        nrOfSegments=0;
        segments.resize(0);
    nrOfIndJoints=0;
    nrOfUnlockedJoints=0;
    locked_joints.resize(0);
        for(unsigned int i=0;i<arg.nrOfSegments;i++)
            addSegment(arg.getSegment(i));
    this->setCoupling(arg.getLockedJoints(),arg.cm);
        return *this;

    }

    void Chain::addSegment(const Segment& segment)
    {
      //std::cout<<"add Segment"<<std::endl;
        segments.push_back(segment);
        nrOfSegments++;
        if(segment.getJoint().getType()!=Joint::NoJoint)
    {
      //std::cout<<"joint"<<std::endl;
            nrOfJoints++;
        locked_joints.push_back(false);
        nrOfUnlockedJoints++;
        cm.setIdentity(nrOfUnlockedJoints,nrOfUnlockedJoints);
        //std::cout<<"test"<<nrOfUnlockedJoints<<std::endl;
        nrOfIndJoints=nrOfUnlockedJoints;
        //std::cout<<cm<<std::endl;
    }

    }

    void Chain::addChain(const Chain& chain)
    {
        for(unsigned int i=0;i<chain.getNrOfSegments();i++)
    {
            this->addSegment(chain.getSegment(i));
        locked_joints.back()=chain.getLockedJoints()[i];
    }
    }

    int Chain::setCoupling(const std::vector<bool> locked_joints,const Eigen::MatrixXd& cm)
    {
        if(locked_joints.size()!=locked_joints.size())
            return -1;
    int nrOfUnlockedJoints_(0);
    //std::cout<<"Locked Joints"<<std::endl;
        for(unsigned int i=0;i<locked_joints.size();i++){
            if(!locked_joints[i])
                nrOfUnlockedJoints_++;
        //std::cout<<locked_joints[i]<<" ";
        }
    //std::cout<<"nrOfunlockedjoints "<<nrOfUnlockedJoints_<<std::endl;
    //std::cout<<"cmrows "<<cm.rows()<<std::endl;

        if(cm.rows()!=nrOfUnlockedJoints_)
      return -1;
        this->locked_joints=locked_joints;
        nrOfUnlockedJoints=nrOfUnlockedJoints_;
    this->cm=cm;
    //std::cout<<this->cm<<std::endl;
    nrOfIndJoints=cm.cols();
    return(0);
    }

    const Segment& Chain::getSegment(unsigned int nr)const
    {
        return segments[nr];
    }

    std::vector<bool> Chain::getLockedJoints() const
    {
      return(locked_joints);
    }


    Chain::~Chain()
    {
    }

}


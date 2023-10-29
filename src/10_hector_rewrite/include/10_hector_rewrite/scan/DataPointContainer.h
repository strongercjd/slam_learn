//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef _DATAPOINTCONTAINER_H_
#define _DATAPOINTCONTAINER_H_

#include <vector>

namespace hectorslam
{
/**
 * @brief 数据点的容器 Container：容器
 * 
 * @tparam DataPointType 数据点的类型
 */
template <typename DataPointType>
class DataPointContainer
{
public:
    DataPointContainer(int size = 1000)
    {
        dataPoints.reserve(size);
    }

    void setFrom(const DataPointContainer &other, float factor)
    {
        origo = other.getOrigo() * factor;

        dataPoints = other.dataPoints;

        unsigned int size = dataPoints.size();

        for (unsigned int i = 0; i < size; ++i)
        {
            dataPoints[i] *= factor;
        }
    }
    /**
     * @brief 添加一个点的数据到容器中
     * 
     * @param dataPoint 坐标，是真实坐标，乘了地图缩放比例。并且是在base_link下的坐标
     */
    void add(const DataPointType &dataPoint)
    {
        dataPoints.push_back(dataPoint);
    }

    void clear()
    {
        dataPoints.clear();
    }

    int getSize() const
    {
        return dataPoints.size();
    }

    const DataPointType &getVecEntry(int index) const
    {
        return dataPoints[index];
    }

    DataPointType getOrigo() const
    {
        return origo;
    }

    void setOrigo(const DataPointType &origoIn)
    {
        origo = origoIn;
    }

protected:
    std::vector<DataPointType> dataPoints;

    /*    
    假设有两个坐标系:
    - base_link 坐标系
    - laser 坐标系
    laser坐标系相对于base_link发生了一个 x=1.0, y=2.0 的位移。
    那么可以创建一个 stamped transform 来表示这个关系:
    tf::StampedTransform laserTransform;
    laserTransform.setOrigin(tf::Vector3(1.0, 2.0, 0.0)); 
    这里设置的原点(1.0, 2.0, 0.0)就是 laser 相对于 base_link 的位移量。
    所以通过 getOrigin() 获得的是 laser 在 base_link 中的坐标位置。
    */
    DataPointType origo;
};

typedef DataPointContainer<Eigen::Vector2f> DataContainer;

} // namespace hectorslam

#endif

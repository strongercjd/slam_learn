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

#ifndef _MAPDIMENSIONPROPERTIES_h_
#define _MAPDIMENSIONPROPERTIES_h_
/**
 * @brief 地图维度属性
 * 
 */
class MapDimensionProperties
{
public:
    MapDimensionProperties()
        : topLeftOffset(-1.0f, -1.0f), mapDimensions(-1, -1), cellLength(-1.0f)
    {
    }

    MapDimensionProperties(const Eigen::Vector2f &topLeftOffsetIn, const Eigen::Vector2i &mapDimensionsIn, float cellLengthIn)
        : topLeftOffset(topLeftOffsetIn), mapDimensions(mapDimensionsIn), cellLength(cellLengthIn)
    {
        mapLimitsf = (mapDimensionsIn.cast<float>()).array() - 1.0f;
    }

    bool operator==(const MapDimensionProperties &other) const
    {
        return (topLeftOffset == other.topLeftOffset) && (mapDimensions == other.mapDimensions) && (cellLength == other.cellLength);
    }

    bool hasEqualDimensionProperties(const MapDimensionProperties &other) const
    {
        return (mapDimensions == other.mapDimensions);
    }

    bool hasEqualTransformationProperties(const MapDimensionProperties &other) const
    {
        return (topLeftOffset == other.topLeftOffset) && (cellLength == other.cellLength);
    }

    bool pointOutOfMapBounds(const Eigen::Vector2f &coords) const
    {
        return ((coords[0] < 0.0f) || (coords[0] > mapLimitsf[0]) || (coords[1] < 0.0f) || (coords[1] > mapLimitsf[1]));
    }

    void setMapCellDims(const Eigen::Vector2i &newDims)
    {
        mapDimensions = newDims;
        /* 将newDims转为浮点数向量，并且每个分量都减去2.0f 
         .cast<float>() 方法可以将Integer Vector 转为 Float Vector
         .array() 方法取得浮点向量的每个分量
         newDims = (100, 80)
         mapLimitsf = (98.0f, 78.0f)
         这里每次减去固定的2.0f ,可能是为了保留边缘给定宽度的网格空间。
        */
        mapLimitsf = (newDims.cast<float>()).array() - 2.0f;
    }

    void setTopLeftOffset(const Eigen::Vector2f &topLeftOffsetIn)
    {
        topLeftOffset = topLeftOffsetIn;
    }

    void setSizeX(int sX) { mapDimensions[0] = sX; };
    void setSizeY(int sY) { mapDimensions[1] = sY; };
    void setCellLength(float cl) { cellLength = cl; };

    const Eigen::Vector2f &getTopLeftOffset() const { return topLeftOffset; };
    const Eigen::Vector2i &getMapDimensions() const { return mapDimensions; };
    int getSizeX() const { return mapDimensions[0]; };
    int getSizeY() const { return mapDimensions[1]; };
    float getCellLength() const { return cellLength; };

protected:
    Eigen::Vector2f topLeftOffset; // 起始坐标，这里我们认为这地图左上角的位置就是世界坐标系的原点
    Eigen::Vector2i mapDimensions;//地图的尺寸，也就是地图的大小,默认是-1
    Eigen::Vector2f mapLimitsf; // 地质限制的尺寸，度点数，比mapDimensions向量小2.0f。这里每次减去固定的2.0f ,可能是为了保留边缘给定宽度的网格空间。
    float cellLength;           //地图分辨率
};

#endif

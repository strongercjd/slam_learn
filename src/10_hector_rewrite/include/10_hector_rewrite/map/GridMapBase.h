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

#ifndef _GRIDMAPBASE_h_
#define _GRIDMAPBASE_h_

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "MapDimensionProperties.h"

namespace hectorslam
{

/**
 * GridMapBase provides basic grid map functionality (creates grid , provides transformation from/to world coordinates).
 * It serves as the base class for different map representations that may extend it's functionality.
 */
template <typename ConcreteCellType>
class GridMapBase
{

public:
    /*
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 是一个Eigen宏,用来给类添加新的操作符overload。
    该宏的作用是:
    - 为类添加新的操作符 operator new() 和 operator new[]() 。
    - 这两个新的操作符会强制分配具有特定 alignment 的内存。
    使用该宏可以保证Eigen向量和矩阵在内存中具有良好的对齐,有助于提高计算性能
    */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief 创建网格表示和转换。
     * 
     * @param mapResolution 地图分辨率
     * @param size 地图大小
     * @param offset 起始坐标
     */
    GridMapBase(float mapResolution, const Eigen::Vector2i &size,
                const Eigen::Vector2f &offset)
        : mapArray(0), lastUpdateIndex(-1)
    {
        Eigen::Vector2i newMapDimensions(size);//新地图的尺寸 Dimensions:尺寸
        this->setMapGridSize(newMapDimensions);
        sizeX = size[0];

        setMapTransformation(offset, mapResolution);

        this->clear();
    }

    /**
   * Destructor
   */
    virtual ~GridMapBase()
    {
        deleteArray();
    }

    /**
   * Indicates if given x and y are within map bounds
   * @return True if coordinates are within map bounds
   */
    bool hasGridValue(int x, int y) const
    {
        return (x >= 0) && (y >= 0) && (x < this->getSizeX()) && (y < this->getSizeY());
    }

    const Eigen::Vector2i &getMapDimensions() const { return mapDimensionProperties.getMapDimensions(); };
    int getSizeX() const { return mapDimensionProperties.getSizeX(); };
    int getSizeY() const { return mapDimensionProperties.getSizeY(); };

    bool pointOutOfMapBounds(const Eigen::Vector2f &pointMapCoords) const
    {
        return mapDimensionProperties.pointOutOfMapBounds(pointMapCoords);
    }

    virtual void reset()
    {
        this->clear();
    }

    /**
   * Resets the grid cell values by using the resetGridCell() function.
   */
    void clear()
    {
        int size = this->getSizeX() * this->getSizeY();

        for (int i = 0; i < size; ++i)
        {
            this->mapArray[i].resetGridCell();
        }

        //this->mapArray[0].set(1.0f);
        //this->mapArray[size-1].set(1.0f);
    }

    const MapDimensionProperties &getMapDimProperties() const { return mapDimensionProperties; };

    /**
   * Allocates memory for the two dimensional pointer array for map representation.
   */
    void allocateArray(const Eigen::Vector2i &newMapDims)
    {
        int sizeX = newMapDims.x();
        int sizeY = newMapDims.y();

        mapArray = new ConcreteCellType[sizeX * sizeY];

        mapDimensionProperties.setMapCellDims(newMapDims);
    }

    void deleteArray()
    {
        if (mapArray != 0)
        {

            delete[] mapArray;

            mapArray = 0;
            mapDimensionProperties.setMapCellDims(Eigen::Vector2i(-1, -1));//设置默认地图大小为-1
        }
    }

    ConcreteCellType &getCell(int x, int y)
    {
        return mapArray[y * sizeX + x];
    }

    const ConcreteCellType &getCell(int x, int y) const
    {
        return mapArray[y * sizeX + x];
    }

    ConcreteCellType &getCell(int index)
    {
        return mapArray[index];
    }

    const ConcreteCellType &getCell(int index) const
    {
        return mapArray[index];
    }
    /**
     * @brief 设置地图大小
     * 
     * @param newMapDims 地图大小
     */
    void setMapGridSize(const Eigen::Vector2i &newMapDims)
    {
        // std::cout<<"X:"<<mapDimensionProperties.getMapDimensions().x()<<std::endl;
        // std::cout<<"Y:"<<mapDimensionProperties.getMapDimensions().y()<<std::endl;
        if (newMapDims != mapDimensionProperties.getMapDimensions())
        {
            deleteArray();
            allocateArray(newMapDims);
            this->reset();
        }
    }

    /**
   * Copy Constructor, only needed if pointer members are present.
   */
    GridMapBase(const GridMapBase &other)
    {
        allocateArray(other.getMapDimensions());
        *this = other;
    }

    /**
   * Assignment operator, only needed if pointer members are present.
   */
    GridMapBase &operator=(const GridMapBase &other)
    {
        if (!(this->mapDimensionProperties == other.mapDimensionProperties))
        {
            this->setMapGridSize(other.mapDimensionProperties.getMapDimensions());
        }

        this->mapDimensionProperties = other.mapDimensionProperties;

        this->worldTmap = other.worldTmap;
        this->mapTworld = other.mapTworld;
        this->worldTmap3D = other.worldTmap3D;

        this->scaleToMap = other.scaleToMap;

        //@todo potential resize
        int sizeX = this->getSizeX();
        int sizeY = this->getSizeY();

        size_t concreteCellSize = sizeof(ConcreteCellType);

        memcpy(this->mapArray, other.mapArray, sizeX * sizeY * concreteCellSize);

        return *this;
    }

    /**
   * Returns the world coordinates for the given map coords.
   */
    inline Eigen::Vector2f getWorldCoords(const Eigen::Vector2f &mapCoords) const
    {
        return worldTmap * mapCoords;
    }

    /**
   * Returns the map coordinates for the given world coords.
   */
    inline Eigen::Vector2f getMapCoords(const Eigen::Vector2f &worldCoords) const
    {
        return mapTworld * worldCoords;
    }

    /**
   * Returns the world pose for the given map pose.
   */
    inline Eigen::Vector3f getWorldCoordsPose(const Eigen::Vector3f &mapPose) const
    {
        Eigen::Vector2f worldCoords(worldTmap * mapPose.head<2>());
        return Eigen::Vector3f(worldCoords[0], worldCoords[1], mapPose[2]);
    }

    /**
   * Returns the map pose for the given world pose.
   */
    inline Eigen::Vector3f getMapCoordsPose(const Eigen::Vector3f &worldPose) const
    {
        Eigen::Vector2f mapCoords(mapTworld * worldPose.head<2>());
        return Eigen::Vector3f(mapCoords[0], mapCoords[1], worldPose[2]);
    }

    void setDimensionProperties(const Eigen::Vector2f &topLeftOffsetIn, const Eigen::Vector2i &mapDimensionsIn, float cellLengthIn)
    {
        setDimensionProperties(MapDimensionProperties(topLeftOffsetIn, mapDimensionsIn, cellLengthIn));
    }

    void setDimensionProperties(const MapDimensionProperties &newMapDimProps)
    {
        //Grid map cell number has changed
        if (!newMapDimProps.hasEqualDimensionProperties(this->mapDimensionProperties))
        {
            this->setMapGridSize(newMapDimProps.getMapDimensions());
        }

        //Grid map transformation/cell size has changed
        if (!newMapDimProps.hasEqualTransformationProperties(this->mapDimensionProperties))
        {
            this->setMapTransformation(newMapDimProps.getTopLeftOffset(), newMapDimProps.getCellLength());
        }
    }

    /**
     * @brief 设置地图变换
     * 
     * @param topLeftOffset 起始坐标
     * @param cellLength 地图分辨率
     */
    void setMapTransformation(const Eigen::Vector2f &topLeftOffset,
                              float cellLength)
    {
        mapDimensionProperties.setCellLength(cellLength);//设置分辨率
        mapDimensionProperties.setTopLeftOffset(topLeftOffset);//设置起始坐标，就是地图左上角的偏移

        scaleToMap = 1.0f / cellLength;//地图分辨率的倒数，也就是地图的缩放比例
        /*
        假设单元长度为0.5米,地图左上角在世界坐标系为 (2,3) 。
        则:
        - scaleToMap 为 1/0.5 = 2
        - topLeftOffset 为 (2,3)
        则变换矩阵为:
        mapTworld = Eigen::AlignedScaling2f(2, 2) * Eigen::Translation2f(2,3);
        Eigen::AlignedScaling2f(2, 2)将地图放大2倍
        Eigen::Translation2f(2,3) 平移到(2,3),也就是平移到世界坐标系下

        则地图坐标 (1,1) 在世界坐标中的位置为:(4,5)

        即地图坐标(1,1) 在世界坐标系下对应的位置为(4,5)。
        */

        mapTworld = Eigen::AlignedScaling2f(scaleToMap, scaleToMap) * Eigen::Translation2f(topLeftOffset[0], topLeftOffset[1]);

        worldTmap3D = Eigen::AlignedScaling3f(scaleToMap, scaleToMap, 1.0f) * Eigen::Translation3f(topLeftOffset[0], topLeftOffset[1], 0);

        //std::cout << worldTmap3D.matrix() << std::endl;
        worldTmap3D = worldTmap3D.inverse();//求逆矩阵

        worldTmap = mapTworld.inverse();
    }

    /**
   * Returns the scale factor for one unit in world coords to one unit in map coords.
   * @return The scale factor
   */
    float getScaleToMap() const
    {
        return scaleToMap;
    }

    /**
   * Returns the cell edge length of grid cells in millimeters.
   * @return the cell edge length in millimeters.
   */
    float getCellLength() const
    {
        return mapDimensionProperties.getCellLength();
    }

    /**
   * Returns a reference to the homogenous 2D transform from map to world coordinates.
   * @return The homogenous 2D transform.
   */
    const Eigen::Affine2f &getWorldTmap() const
    {
        return worldTmap;
    }

    /**
   * Returns a reference to the homogenous 3D transform from map to world coordinates.
   * @return The homogenous 3D transform.
   */
    const Eigen::Affine3f &getWorldTmap3D() const
    {
        return worldTmap3D;
    }

    /**
   * Returns a reference to the homogenous 2D transform from world to map coordinates.
   * @return The homogenous 2D transform.
   */
    const Eigen::Affine2f &getMapTworld() const
    {
        return mapTworld;
    }

    void setUpdated() { lastUpdateIndex++; };
    int getUpdateIndex() const { return lastUpdateIndex; };

    /**
    * Returns the rectangle ([xMin,yMin],[xMax,xMax]) containing non-default cell values
    */
    bool getMapExtends(int &xMax, int &yMax, int &xMin, int &yMin) const
    {
        int lowerStart = -1;
        int upperStart = 10000;

        int xMaxTemp = lowerStart;
        int yMaxTemp = lowerStart;
        int xMinTemp = upperStart;
        int yMinTemp = upperStart;

        int sizeX = this->getSizeX();
        int sizeY = this->getSizeY();

        for (int x = 0; x < sizeX; ++x)
        {
            for (int y = 0; y < sizeY; ++y)
            {
                if (this->mapArray[x][y].getValue() != 0.0f)
                {

                    if (x > xMaxTemp)
                    {
                        xMaxTemp = x;
                    }

                    if (x < xMinTemp)
                    {
                        xMinTemp = x;
                    }

                    if (y > yMaxTemp)
                    {
                        yMaxTemp = y;
                    }

                    if (y < yMinTemp)
                    {
                        yMinTemp = y;
                    }
                }
            }
        }

        if ((xMaxTemp != lowerStart) &&
            (yMaxTemp != lowerStart) &&
            (xMinTemp != upperStart) &&
            (yMinTemp != upperStart))
        {

            xMax = xMaxTemp;
            yMax = yMaxTemp;
            xMin = xMinTemp;
            yMin = yMinTemp;
            return true;
        }
        else
        {
            return false;
        }
    }

protected:
    ConcreteCellType *mapArray; ///< Map representation used with plain pointer array.
    // 世界坐标系(world frame)是一个固定的参考坐标系,通常与机器人固定相对。这里指的是地图左上角位置作为世界坐标系的原点
    // 地图坐标系(map frame)则是用于构建和表示地图的数据坐标系。这里指的是地图左下角位置作为地图坐标系的原点
    float scaleToMap; ///< 世界坐标系到地图坐标系的比例因子，也就是地图分辨率的倒数

    Eigen::Affine2f worldTmap;   ///< Homogenous 2D transform from map to world coordinates.
    Eigen::Affine3f worldTmap3D; ///< Homogenous 3D transform from map to world coordinates.
    Eigen::Affine2f mapTworld;   ///< Homogenous 2D transform from world to map coordinates.

    MapDimensionProperties mapDimensionProperties;
    int sizeX;

private:
    int lastUpdateIndex;
};
}

#endif

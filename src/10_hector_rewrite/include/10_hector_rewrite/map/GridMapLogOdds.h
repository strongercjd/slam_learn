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

#ifndef _GRIDMAPLOGODDS_h_
#define _GRIDMAPLOGODDS_h_

#include <cmath>

/**
 * @brief 提供占用网格地图中单元的占用概率表示的对数赔率。
 * Log Odds (对数几率)是一种表示概率的有效方法。概率本身用0~1之间的小数表示,但是Log Odds用对数比(log ratio)表示概率,计算时更简单高效。
 * Log Odds = log(P/(1-P))
 * P = 1/(1+ e^(-Log Odds)) 
 */
class LogOddsCell
{
public:

    /**
   * Sets the cell value to val.
   * @param val The log odds value.
   */
    void set(float val)
    {
        logOddsVal = val;
    }

    /**
   * Returns the value of the cell.
   * @return The log odds value.
   */
    float getValue() const
    {
        return logOddsVal;
    }

    /**
   * Returns wether the cell is occupied.
   * @return Cell is occupied
   */
    bool isOccupied() const
    {
        return logOddsVal > 0.0f;
    }

    bool isFree() const
    {
        return logOddsVal < 0.0f;
    }

    /**
   * Reset Cell to prior probability.
   */
    void resetGridCell()
    {
        logOddsVal = 0.0f;
        updateIndex = -1;
    }

    //protected:

public:
    float logOddsVal; ///< The log odds representation of occupancy probability.
    int updateIndex;
};

/**
 * @brief 提供与占用网格地图中单元的占用概率重新呈现的对数赔率相关的函数。
 * Log Odds (对数几率)是一种表示概率的有效方法。概率本身用0~1之间的小数表示,但是Log Odds用对数比(log ratio)表示概率,计算时更简单高效。
 * Log Odds = log(P/(1-P))
 * P = 1/(1+ e^(-Log Odds)) 
 * 
 */
class GridMapLogOddsFunctions
{
public:
    /**
   * Constructor, sets parameters like free and occupied log odds ratios.
   */
    GridMapLogOddsFunctions()
    {
        this->setUpdateFreeFactor(0.4f);
        this->setUpdateOccupiedFactor(0.6f);
    }

    /**
   * Update cell as occupied
   * @param cell The cell.
   */
    void updateSetOccupied(LogOddsCell &cell) const
    {
        if (cell.logOddsVal < 50.0f)
        {
            cell.logOddsVal += logOddsOccupied;
        }
    }

    /**
   * Update cell as free
   * @param cell The cell.
   */
    void updateSetFree(LogOddsCell &cell) const
    {

        cell.logOddsVal += logOddsFree;
    }

    void updateUnsetFree(LogOddsCell &cell) const
    {
        cell.logOddsVal -= logOddsFree;
    }

    /**
   * Get the probability value represented by the grid cell.
   * @param cell The cell.
   * @return The probability
   */
    float getGridProbability(const LogOddsCell &cell) const
    {
        float odds = exp(cell.logOddsVal);
        return odds / (odds + 1.0f);
    }

    void setUpdateFreeFactor(float factor)
    {
        logOddsFree = probToLogOdds(factor);
    }

    void setUpdateOccupiedFactor(float factor)
    {
        logOddsOccupied = probToLogOdds(factor);
    }

protected:
    float probToLogOdds(float prob)
    {
        float odds = prob / (1.0f - prob);
        return log(odds);
    }

    float logOddsOccupied; /// < The log odds representation of probability used for updating cells as occupied
    float logOddsFree;     /// < The log odds representation of probability used for updating cells as free
};

#endif

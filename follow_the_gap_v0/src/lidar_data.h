/*
 *
 * LidarData class (header)
 *
 * Authors: Jaroslav Klapálek
 * Copyright (C) 2020 Czech Technical University in Prague
 *
 * This file is a part of follow_the_gap_v0.
 *
 * follow_the_gap_v0 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * follow_the_gap_v0 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with follow_the_gap_v0. If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifndef LIDAR_DATA_H_
#define LIDAR_DATA_H_

class LidarData {
    public:
        float range_min = 0.3;
        float range_max = 8;

        float angle_min = -2.35 ;
        float angle_max = 2.35;

        float angle_increment = 0.026256984;

        LidarData(float range_min, float range_max, float angle_min, float angle_max, float angle_increment);
};

#endif

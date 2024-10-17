/*
 *    Copyright (C) 2024 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <expected>
#include <random>


class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

    public slots:
        void initialize();
        void compute();
        void emergency();
        void restore();
        int startup_check();

    private:
        struct Params
        {
            float ROBOT_WIDTH = 460;  // mm
            float ROBOT_LENGTH = 480;  // mm
            float MAX_ADV_SPEED = 1000; // mm/s
            float MAX_ROT_SPEED = 1; // rad/s
            float STOP_THRESHOLD = MAX_ADV_SPEED*0.5; // mm  parar antes de chocar
            float ADVANCE_THRESHOLD = ROBOT_WIDTH * 1.4; // mm recuperar después de chocar
            float LIDAR_FRONT_SECTION = 0.5; // rads, aprox 30 degrees
            float WALL_DESIRED_DISTANCE = 400;
            float DELTA = 10;

            std::string LIDAR_NAME_LOW = "bpearl";
            std::string LIDAR_NAME_HIGH = "helios";
            QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

        };
        Params params;

        bool startup_check_flag;
        AbstractGraphicViewer *viewer;

        // state machine
        enum class STATE {FORWARD, TURN, WALL, SPIRAL };
        STATE state = STATE::FORWARD;

        using RetVal = std::tuple<STATE, float, float>;
        RetVal forward(auto &filtered_points);
        RetVal turn(auto &filtered_points);
        RetVal wall(auto &filtered_points);
        RetVal spiral(auto &filtered_points);
        void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
        QGraphicsPolygonItem* robot_draw;
        std::expected<int, string> closest_lidar_index_to_given_angle(const auto &points, float angle);

        // random number generator
        std::random_device rd;
};

#endif

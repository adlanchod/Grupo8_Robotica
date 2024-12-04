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
#include "Lidar3D.h"
#include <expected>
#include <random>
#include <doublebuffer_sync/doublebuffer_sync.h>
#include <locale>
#include <qcustomplot/qcustomplot.h>
#include <Eigen/Dense>
#include "Lidar3D.h"
#include <cppitertools/range.hpp>
#include <limits>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompGrid2D::Result Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target);


public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();
	void viewerSlot(QPointF coordinates);
private:
	bool startup_check_flag;
	std::vector<QPointF> path; // Almacena el camino calculado por Dijkstra

	struct Params {
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
	};
	Params params;

	enum class STATE {
		UNKNOWN, FREE, OCCUPIED
	};


	struct TCell {
		STATE state = STATE::UNKNOWN;
		QGraphicsRectItem *item;// Representación gráfica
		int cost = std::numeric_limits<int>::max(); // Costo inicial para Dijkstra
		bool visited = false;         // Indicador para saber si la celda fue visitada
	};
	TCell cell;

	static constexpr  int CELL_SIZE = 100;    // Tamaño de cada celda en mm
	static constexpr  int GRID_LENGTH = 10000; // Dimensión total de la cuadrícula en mm
	static constexpr  int GRID_SIZE = GRID_LENGTH/CELL_SIZE;

	std::array<std::array<TCell, GRID_SIZE>, GRID_SIZE> grid;// Número de celdas por lado

	//Draw
	AbstractGraphicViewer *viewer;
	void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
	QGraphicsPolygonItem *robot_draw;

	QPointF indexToReal(float i, float j);
	QPointF realToIndex(float i, float j);

	void update(const std::vector<Eigen::Vector2f> &lidar_points);
	void clearGrid();
	// Método para calcular el camino óptimo usando Dijkstra
	std::vector<QPointF> dijkstra(const std::pair<int, int> &start, const std::pair<int, int> &goal);
	void draw_path(std::vector<QPointF> path, QGraphicsScene *scene);


	// lidar
	std::vector<Eigen::Vector2f> read_lidar_bpearl();
};

#endif

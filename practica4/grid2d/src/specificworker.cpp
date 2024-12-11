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
#include "specificworker.h"
#include <cppitertools/enumerate.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }
	

	return true;
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame_lidar, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		viewer->setStyleSheet("background-color: lightGray;");
		this->resize(800, 700);

		QPen pen (QColor("blue"), 20);
		for (const auto &[i, row] : grid | iter::enumerate)
		{
			for (const auto &[j, cell] : row | iter::enumerate)
			{
				cell.state = STATE::UNKNOWN;
				cell.item= viewer->scene.addRect(-CELL_SIZE/2, CELL_SIZE/2,CELL_SIZE, CELL_SIZE, pen);
				cell.item->setPos(indexToReal(i, j));
			}
		}


		connect(viewer, SIGNAL(new_mouse_coordinates(QPointF)), this, SLOT(viewerSlot(QPointF)));


		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);
		viewer->show();

	}

}

QPointF SpecificWorker::indexToReal(float i, float j)
{
	float x = ((GRID_LENGTH /GRID_SIZE) * i)-GRID_LENGTH/2;
	float y = -((GRID_LENGTH /GRID_SIZE) * j)+GRID_LENGTH/2;

	return QPointF(x,y);

}
QPointF SpecificWorker::realToIndex(float i, float j)
{

	float scale = GRID_LENGTH/static_cast<float>(GRID_SIZE);

	float x = (i + GRID_LENGTH/2)/scale;
	float y = (GRID_LENGTH / 2 - j) / scale;

	return QPointF(x,y);
}

void SpecificWorker::compute()
{
    std::cout << "Compute worker" << std::endl;
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
    //    if (img.empty())
    //        emit goToEmergency()
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	auto ldata_bpearl = read_lidar_bpearl();
	if(ldata_bpearl.empty()) { qWarning() << __FUNCTION__ << "Empty bpearl lidar data"; return; };
	draw_lidar(ldata_bpearl, &viewer->scene);
	clearGrid();
	update(ldata_bpearl);
	draw_path(path, &viewer->scene);



}

std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_bpearl()
{
	try
	{
		auto ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
		// filter points according to height and distance
		std::vector<Eigen::Vector2f>  p_filter;
		for(const auto &a: ldata.points)
		{
			if(a.z < 500 and a.distance2d > 200)
				p_filter.emplace_back(a.x, a.y);
		}
		return p_filter;
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
	return {};
}

void SpecificWorker::clearGrid() {
	QPen pen(QColor("blue"), 20);

	// Recorremos cada celda de la cuadrícula
	for (const auto &[i, row] : grid | iter::enumerate) {
		for (const auto &[j, cell] : row | iter::enumerate) {
			// Restablecer estado a UNKNOWN
			cell.state = STATE::UNKNOWN;

			// Cambiar el color a gris claro
			if (cell.item)
			{
				cell.item->setBrush(QColor(Qt::lightGray));
			}
		}
	}
}

void SpecificWorker::update(const std::vector<Eigen::Vector2f> &lidar_points)
{
	//Recorremos cada uno de los puntos del lidar:
	for (const auto &point : lidar_points)
	{
		float n = point.norm() / CELL_SIZE;
		for (auto s: iter::range(0.f,1.f, 1.f/n))
		{
			QPointF  index = realToIndex(s * point.x(), s * point.y());

			//Convertimos los números de las celdas a números enteros:
			int i = static_cast<int>(index.x());
			int j = static_cast<int>(index.y());


			// Verifica si los índices están dentro de los límites de la cuadrícula
			if (i >= 0 && i < GRID_SIZE && j >= 0 && j < GRID_SIZE)
			{
				if (s + 1.f/n < 1.0)
				{

					// Marca la celda como libre (blanco) si no es el último punto
					grid[i][j].state = STATE::FREE;
					grid[i][j].item->setBrush(QColor(Qt::white));
				}
				else
				{

					// Marca la última celda como ocupada (rojo)
					grid[i][j].state = STATE::OCCUPIED;
					grid[i][j].item->setBrush(QColor(Qt::red));

					// Pintamos las celdas vecinas
					for (int dx = -2; dx <= 2; dx++)
					{
						for (int dy = -2; dy <= 2; dy++)
						{
							int nx = i + dx;
							int ny = j + dy;

							// Comprobamos que las celdas vecinas estén dentro de la cuadrícula
							if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE)
							{
								// Pintamos la celda vecina (por ejemplo, en rojo también)
								grid[nx][ny].state = STATE::OCCUPIED;
								grid[nx][ny].item->setBrush(QColor(Qt::red));
							}
						}
					}
				}
			}
		}
	}
}

std::vector<QPointF> SpecificWorker::dijkstra(const std::pair<int, int> &start, const std::pair<int, int> &goal)
{
    // Inicializar la cuadrícula para el algoritmo
    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            grid[i][j].cost = std::numeric_limits<int>::max(); // Costo inicial infinito
            grid[i][j].visited = false;                       // Todas las celdas están sin visitar
        }
    }

    // Cola de prioridad para procesar las celdas (min-heap)
    using QueueElement = std::tuple<int, int, int>; // (costo acumulado, x, y)
    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> pq;

    // Establecer el punto de inicio
    int startX = start.first;
    int startY = start.second;
    grid[startX][startY].cost = 0;
    pq.push({0, startX, startY});

    // Mapa para rastrear el predecesor de cada celda
    std::array<std::array<std::pair<int, int>, GRID_SIZE>, GRID_SIZE> predecessors;
    for (auto &row : predecessors)
        for (auto &cell : row)
            cell = {-1, -1}; // Sin predecesor al inicio

    // Direcciones de movimiento (arriba, abajo, izquierda, derecha)
    const std::vector<std::pair<int, int>> directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}};

    // Procesar las celdas hasta que la cola esté vacía
    while (!pq.empty())
    {
        auto [currentCost, x, y] = pq.top();
        pq.pop();

        // Si ya se visitó, omitir
        if (grid[x][y].visited)
            continue;

        grid[x][y].visited = true;

        // Si llegamos al objetivo, reconstruir la ruta
        if (x == goal.first && y == goal.second)
        {
            std::vector<std::pair<int, int>> path;
            for (auto p = goal; p != start; p = predecessors[p.first][p.second])
                path.push_back(p);
            path.push_back(start);
            std::reverse(path.begin(), path.end());
			std::vector<QPointF> rpath;
        	for (const auto i: path)
        		rpath.emplace_back(indexToReal(i.first, i.second));
            return rpath;
        }

        // Explorar las celdas vecinas
        for (const auto &[dx, dy] : directions)
        {
            int nx = x + dx;
            int ny = y + dy;

            // Verificar si la celda vecina es válida
            if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE && grid[nx][ny].state != STATE::OCCUPIED)
            {
                int newCost = currentCost + 1; // Cada paso tiene un costo de 1
                if (newCost < grid[nx][ny].cost)
                {
                    grid[nx][ny].cost = newCost;
                    pq.push({newCost, nx, ny});
                    predecessors[nx][ny] = {x, y};
                }
            }
        }
    }
    // Si no se encontró una ruta, devolver una ruta vacía
    return {};
}

void SpecificWorker::viewerSlot(QPointF coordinates)
{
	// Mostrar las coordenadas reales recibidas
	qDebug() << "Coordenadas reales pulsadas:" << coordinates;

	// Convertir coordenadas reales a índices de la cuadrícula
	QPointF index = realToIndex(coordinates.x(), coordinates.y());
	int goalX = static_cast<int>(index.x());
	int goalY = static_cast<int>(index.y());

	qDebug() << "Indices obtenidos:" << goalX << goalY;
	// Verificar que los índices estén dentro del rango de la cuadrícula
	if (goalX < 0 || goalX >= GRID_SIZE || goalY < 0 || goalY >= GRID_SIZE)
	{
		qDebug() << "El punto seleccionado está fuera de los límites de la cuadrícula.";
		return;
	}

	qDebug() << "Índices de cuadrícula objetivo:" << goalX << goalY;

	// Establecer el punto de inicio como el centro de la cuadrícula
	int startX = GRID_SIZE / 2;
	int startY = GRID_SIZE / 2;

	// Ejecutar el algoritmo de Dijkstra
	 path = dijkstra({startX, startY}, {goalX, goalY});


}


void SpecificWorker::draw_path(std::vector<QPointF> path, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	auto color = QColor(QColor(128, 0, 128));
	auto brush = QBrush(QColor(128, 0, 128));
	for(const auto &p : path)
	{
		auto item = scene->addRect(-50, -50, 100, 100, color, brush);
		item->setPos(p.x(), p.y());
		items.push_back(item);
	}
}
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	auto color = QColor(Qt::darkGreen);
	auto brush = QBrush(QColor(Qt::darkGreen));
	for(const auto &p : filtered_points)
	{
		auto item = scene->addRect(-50, -50, 100, 100, color, brush);
		item->setPos(p.x(), p.y());
		items.push_back(item);
	}
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}


RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{

	RoboCompGrid2D::Result result;

	// Convertir los puntos de entrada al formato necesario para dijkstra
	std::pair<int, int> start = {source.x, source.y};
	std::pair<int, int> goal = {target.x, target.y};

	// Llamar al algoritmo Dijkstra para calcular la ruta
	auto path = dijkstra(start, goal);
	std::ranges::transform(path, std::back_inserter(result.path), [](auto &p) { return RoboCompGrid2D::TPoint{p.x(), p.y(), 0.f};});
	return result;
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData

/**************************************/
// From the RoboCompGrid2D you can use this types:
// RoboCompGrid2D::TPoint
// RoboCompGrid2D::Result


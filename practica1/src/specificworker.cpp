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

		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);
	}
}

void SpecificWorker::compute()
{
    std::cout << "Compute worker" << std::endl;

	// get laser data from Laser proxy
	RoboCompLaser::TLaserData ldata;
	try{ ldata = laser_proxy->getLaserData();}
	catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; }


	//qDebug() << "Mind dist:" <<  m->dist;

	// state machine to control de robot

	 switch (estado)
	 {
	 	case ESTADO::avanzar:
	 		avanzar(ldata);
		case ESTADO::rotar:
	 		rotar(ldata);
	//

	 }




	// control the robot
	//float adv = 5000;
	//float rot = 0;
	//try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	//catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; }

}

//////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::avanzar(const RoboCompLaser::TLaserData &ldata)
{
	// exit condition
	// check obstacle on front

	float  pos_inicial= ldata.size()/2;
	float	inicio = pos_inicial - ldata.size()/8;
	float final = pos_inicial + ldata.size()/8;

	auto m = std::min_element(ldata.begin() + inicio, ldata.begin() + final, [](const auto &x, const auto &y)
		{ return x.dist < y.dist;});

	std::cout << "Distancia mínima: " << m->dist << std::endl;

	if (m->dist<=400)
	{
		// parar robot
		try{ omnirobot_proxy->setSpeedBase(0, 0, 0);}
		catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; }
		estado = ESTADO::rotar;
	}

	// do my thing
	float adv = 5000;
	float rot = 0;
	try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; }
}

void SpecificWorker::rotar(const RoboCompLaser::TLaserData &ldata) {
	std::cout << "Voy a rotar: " << std::endl;


	float adv = 0;
	float rot = 0;
	try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	catch(const Ice::Exception &e){ std::cout << e.what() << std::endl; }

}
/////////////////////////////////////////////////////////////////////////////////////

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




/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams


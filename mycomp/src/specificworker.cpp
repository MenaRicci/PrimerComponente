/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{



	
	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
     const float threshold = 500; //Distancia previa a choque
    float rot = 1.5707;  //



    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;


     if( ldata.front().dist < threshold + 50)
    {
    differentialrobot_proxy->setSpeedBase(50, rot);
    usleep(1250000);
    std::cout << ldata.front().dist << std::endl;   
        std::cout << "Girando" << std::endl;   
    differentialrobot_proxy->setSpeedBase(200, 0); // sprint
    usleep(500000);
    rot = rot + 0.12; // 0.12
    if( rot > 3 * 1.5707 )
    {
     rot = 1.5707;//1.5707
    }
    }

    else
    {
    differentialrobot_proxy->setSpeedBase(500, 0); 
    usleep(500000);
    std::cout << ldata.front().dist << std::endl;
    }


    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
  
}








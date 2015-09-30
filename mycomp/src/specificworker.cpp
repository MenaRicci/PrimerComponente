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
      const float threshold = 550; //Distancia previa a choque
      float rot = 0.7707;  //
      const float acot = 20;
      //static QTime reloj = QTime::currentTime();
      //static QTime relojAux= QTime::currentTime();
     static int num_giros=0;
    
    try
    {
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin()+ acot , ldata.end() - acot  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;


     if((ldata.data()+30)->dist < threshold )
    {
     //  relojAux.addSecs(1);
     //  if(reloj<relojAux){
	// and (reloj<relojAux)
	 
      if((ldata.data()+30)->angle >= 0 )
      {
	num_giros++;
	differentialrobot_proxy->setSpeedBase(5, -rot);
      }
    
      else {
	num_giros++;
	differentialrobot_proxy->setSpeedBase(5, rot);
      }
       
      if (num_giros > 3 or (ldata.data()+30)->dist < threshold){
    differentialrobot_proxy->setSpeedBase(5, -rot);
       differentialrobot_proxy->setSpeedBase(5, -rot);
      differentialrobot_proxy->setSpeedBase(5, -rot);
       std::cout << num_giros << std::endl;
      num_giros=0;
               std::cout << "---------------" << std::endl;
       } 
       //usleep(1250000);
	std::cout << ldata.front().dist << std::endl;   
        std::cout << "Girando" << std::endl;  
    }
    else
    {
    num_giros=0;
      differentialrobot_proxy->setSpeedBase(500, 0); 
    std::cout << ldata.front().dist << std::endl;
    }


    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
  
}








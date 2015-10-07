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
     static float sentido_giro;
    
    try
    {

	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
        std::sort( ldata.begin()+ acot , ldata.end() - acot  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;


     if((ldata.data()+30)->dist < threshold )
    {
	 
      if((ldata.data()+30)->angle >= 0 )
      {
	sentido_giro=-rot;
	differentialrobot_proxy->setSpeedBase(0, -rot);
      }
    
      else {
	sentido_giro=rot;
	differentialrobot_proxy->setSpeedBase(0, rot);
      }
      
      
      int  numero = rand() % 15;
      if(numero %3==0){
	differentialrobot_proxy->setSpeedBase(0, sentido_giro*3);
	usleep(250000);
      }
    
       usleep(125000);
	std::cout << ldata.front().dist << std::endl;   
        std::cout << "Girando" << std::endl;  
    }
    else
    {
      differentialrobot_proxy->setSpeedBase(500, 0); 
      std::cout << ldata.front().dist << std::endl;
    }


    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
  
}



int SpecificWorker::obtener_dato()
{

  
return 0;
  
}
	

SpecificWorker::DatosCamara::Tag SpecificWorker::DatosCamara::get(){

  DatosCamara::Tag A;
  return A;
}

void SpecificWorker::DatosCamara::add(tag T)
{
  lista.push_back(T);
}

////////////////////////////////////////////////////////////77
//////  EN EL HILO THE ICE
////////////////////////////////////////////////////////////

void SpecificWorker::newAprilTag(const tagsList &tags){
  
    for(auto t : tags)
      qDebug() << t.id;	
    
    
    //marcas.lista=tags;
   marcas.lista.push_back(tags.front());
   // DatosCamara::MyTag a;
   //marcas.add(tags);
    
}
  







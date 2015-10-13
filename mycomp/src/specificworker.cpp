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
    try
    {
	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
    
    
    
    switch(state)
    {
      case State::INIT:
	
	state = State::SEARCH;
	break;
      case State::SEARCH:
	search();
	if(encontrado!=0)
	  state = State::ADVANCE;
	break;
      case State::ADVANCE:
	avanzar(ldata);
	if((ldata.data()+30)->dist < 400){
	  recorrido++;
	  if(recorrido==4)
	    state = State::STOP;
	  else{
	   encontrado=0;
	   state = State::SEARCH;
	  }
	}
	break;
      case State::STOP:
	//FIN
	break;
    }
    }
catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }	 
}


void SpecificWorker::search()
{
  if(!marcas.lista.empty()){
    DatosCamara::MyTag A =copia(marcas.lista.front());
    marcas.lista.pop_back();
  if(A.id==recorrido)
  {
    DatoEncontrado=A;
    encontrado++; 
  }
    
    
  }else
    differentialrobot_proxy->setSpeedBase(0, 0.77);
    usleep(10000);

}
  


SpecificWorker::DatosCamara::MyTag SpecificWorker::copia(tag T )
{
     DatosCamara::MyTag A;
     A.id=T.id;
     A.dist_x=T.tx;
     A.dist_y=T.ty;
     A.dist_z=T.tz;
     A.rot_x=T.rx;
     A.rot_y=T.ry;
     A.rot_z=T.rz;
     
  return A;
  
}


void SpecificWorker::avanzar( RoboCompLaser::TLaserData copiaLaser)
{
    const float threshold = 550; //Distancia previa a choque
      float rot = 0.7707;  //
      const float acot = 20;
     static float sentido_giro;
    
      std::sort( copiaLaser.begin()+ acot , copiaLaser.end() - acot  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;


     if((copiaLaser.data()+30)->dist < threshold )
    {
	 
      if((copiaLaser.data()+30)->angle >= 0 )
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
	std::cout << copiaLaser.front().dist << std::endl;   
        std::cout << "Girando" << std::endl;  
    }
    else
    {
      differentialrobot_proxy->setSpeedBase(500, 0); 
      std::cout << copiaLaser.front().dist << std::endl;
    }


    }
 
	

SpecificWorker::DatosCamara::MyTag SpecificWorker::DatosCamara::get(){

  DatosCamara::MyTag A;
  
 // A=marcas.lista.front();
  return A;
  
}

////////////////////////////////////////////////////////////77
//////  EN EL HILO THE ICE
////////////////////////////////////////////////////////////

void SpecificWorker::newAprilTag(const tagsList &tags){
  
    for(auto t : tags)
    {
      if(t.id == recorrido)
	marcas.lista.push_back(t);
     // qDebug() << t.id;	
      
    }
    
    
    //marcas.lista=tags;
   //(tags.front());
   // DatosCamara::MyTag a;
   //marcas.add(tags);
    
}
  







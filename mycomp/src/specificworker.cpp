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
#include <qt4/QtCore/qlist.h>

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
    
	DatosCamara::MyTag B;
      float distancia_P;
    switch(state)
    {
      case State::INIT:
	    //usleep(10000);
	state = State::SEARCH;
	break;
      case State::SEARCH:
	search();
	std::cout << "<--Buscar-->"<< std::endl;
	break;
      case State::ADVANCE:
	std::cout << "<--Avance-->"<< std::endl;
	if(contains(recorrido)){
	   // avanzar(ldata);
	    std::cout << "Valor recorrido: " <<recorrido<< std::endl;  
	    B=get(recorrido); 
	    std::cout << "ID Dato: " <<B.id<< std::endl;
	    std::cout << "Rotacion X Dato: " <<B.rot_x<< std::endl;
	    std::cout << "Distancia Y Dato: " <<B.dist_y<< std::endl;
	    std::cout << "Distancia Z Dato: " <<B.dist_z<< std::endl;
	    distancia_P=sqrt(pow(B.dist_x,2) + pow(B.dist_z,2));
	    std::cout << "Distancia al objetivo: " <<distancia_P<< std::endl;
	    if(distancia_P < 300){
	      differentialrobot_proxy->setSpeedBase(0,0);
	      recorrido++;
		if(recorrido==4)
		  state = State::STOP;
		else{
		  encontrado=0;
		  state = State::SEARCH;
	      }
	    }else
	      encontrado=0;
	      state = State::SEARCH;
	}
	avanzar(ldata);
	break;
      case State::STOP:
	differentialrobot_proxy->setSpeedBase(0,0);
	std::cout << "<--FIN-->"<< std::endl;   
	
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
  static bool Primero=true;
  if(contains(recorrido)){
  
    differentialrobot_proxy->setSpeedBase(0, 0);
    state=State::ADVANCE;
    Primero=true;
    encontrado++;
    
  }if(Primero){
    differentialrobot_proxy->setSpeedBase(0, 0.77);
    usleep(10000);
    Primero=false;
    encontrado=0;
    
  }
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

SpecificWorker::DatosCamara::MyTag SpecificWorker::get(int id)
{
  DatosCamara::MyTag A;
  
  for(auto t : marcas.lista){
    if(t.id==id){
      A=copia(t);
      return A;
    }
  }return A;
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
	state = State::SEARCH;	
	encontrado=0;
      if((copiaLaser.data()+30)->angle >= 0 )
      {

	sentido_giro=-rot;
	differentialrobot_proxy->setSpeedBase(100, -rot);
      }
    
      else {
	
	sentido_giro=rot;
	differentialrobot_proxy->setSpeedBase(100, rot);
      }
      
      
       int  numero = rand() % 15;
       if(numero %3==0){
	  differentialrobot_proxy->setSpeedBase(0, sentido_giro*3);
	  usleep(250000);
      }
        
       usleep(125000);
	//std::cout << copiaLaser.front().dist << std::endl;   
       // std::cout << "Girando" << std::endl;  
    }
    else
    {
      differentialrobot_proxy->setSpeedBase(500, 0); 
     // std::cout << copiaLaser.front().dist << std::endl;
    }


    }
    
    
int SpecificWorker::contains(int id)
{
for(auto t : marcas.lista)
{
  if(t.id==id)
    return 1;
}
  return 0;
}

 
 
////////////////////////////////////////////////////////////77
//////  EN EL HILO THE ICE
////////////////////////////////////////////////////////////

void SpecificWorker::newAprilTag(const tagsList &tags){
  
    for(auto t : tags)
    {
     
     // if(t.id == recorrido)
	marcas.lista.push_back(t);
      
    }
        
}
  







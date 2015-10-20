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
   inner= new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
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
    
	//DatosCamara::MyTag B;
    switch(state)
    {
      case State::INIT:
	    //usleep(10000);
	
	if(recorrido == 4 ){
	  differentialrobot_proxy->setSpeedBase(0,0);  
	  sleep(1000000);
	}
	avanzar(ldata);
	break;
      case State::SEARCH:
	search(ldata);
	//std::cout << "<--Buscar-->"<< std::endl;
	break;
      case State::ADVANCE:
	
	  differentialrobot_proxy->getBaseState(tbase);
	  inner->updateTransformValues("base",tbase.x,0,tbase.z,0,tbase.alpha,0);
	state=State::INIT;
 	//avanzar2(ldata);
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



void SpecificWorker::avanzar2(RoboCompLaser::TLaserData copiaLaser)
{
	float distancia_P;
      float rot;
      DatosCamara::MyTag B;
	std::cout << "<--Avance-->"<< std::endl;
	if(contains(recorrido)){
// 	    avanzar(ldata);
	    
	   // std::cout << "Valor recorrido: " <<recorrido<< std::endl;  
	    B=get(recorrido); 
	    marcas.lista.clear();
	    
	    
	    rot=atan2(B.dist_x,B.dist_z);
	    differentialrobot_proxy->setSpeedBase(300,rot);
	    
	    distancia_P=sqrt(pow(B.dist_x,2) + pow(B.dist_z,2));
	   // std::cout << "Distancia al objetivo: " <<distancia_P<< std::endl;
	    if(distancia_P < 800){
	      differentialrobot_proxy->setSpeedBase(0,0);
	      recorrido++;
	       std::cout << "------------------------------------------------------------------------------------------------ +  1 "<< std::endl;
		if(recorrido==4)
		  state = State::STOP;
		sleep(2);
	    }
	}else{
	      encontrado=0;
 	      marcas.lista.clear();
	}
	state = State::INIT;
}

void SpecificWorker::search(RoboCompLaser::TLaserData copiaLaser)
{
  
  differentialrobot_proxy->setSpeedBase(0, 0);
  
  if(contains(recorrido)){
    
      //Acutalizar Marca en memoria de inner
     differentialrobot_proxy->getBaseState(tbase);
    inner->updateTransformValues("base",tbase.x,0,tbase.z,0,tbase.alpha,0);
 
    p = inner->transform("world",QVec::vec3(),"rgbd");
    //::vec3(2,4,8);
    
    std::cout << "<--Encontrado-->"<< std::endl;   
    differentialrobot_proxy->setSpeedBase(0, 0);
    avanzar2(copiaLaser);
    encontrado++;
    
  }else{
    
   //  if (Comprobar si esta en memoria en el inner== true)
	  
	  //Dirigirnos hacia marca
   
    
    //else --> Buscar == Avanzar()
    
    
    
    
    
    
    encontrado=0;
    state = State::INIT;

    
  }
}
  
void SpecificWorker::NoEncontrado(RoboCompLaser::TLaserData copiaLaser)
{
 const float threshold = 550; //millimeters
    float rotaux,rot = 0.7707;  //rads per second
    int acot=20;
     int  numero = rand() % 3;
     
     switch(numero){
       case 1:
	 rotaux=-1;
	 break;
	case 2:
	 rotaux=0;
	 break;
       case 3:
	 rotaux=1;
	 break;
	 
     }
    
    try
    {
        //RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  
         std::sort( copiaLaser.begin()+ acot , copiaLaser.end() - acot  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;
  
	 
	 if( copiaLaser.data()->dist+30 < threshold)
   {
      //  std::cout << ldata.dat().dist << std::endl;
        differentialrobot_proxy->setSpeedBase(0, rot);
        usleep(250000);  //random wait between 1.5s and 0.1sec
    }
    else
    {
        differentialrobot_proxy->setSpeedBase(250,rotaux ); 
	usleep(250000); 
	
    }
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}


SpecificWorker::DatosCamara::MyTag SpecificWorker::copia(tag T )
{
     DatosCamara::MyTag A;
     A.id=T.id;
     A.dist_x=1000 * T.tx;
     A.dist_y=1000 * T.ty;
     A.dist_z=1000 * T.tz;
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
  	RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();

    const float threshold = 400; //Distancia previa a choque
      float rot = 0.7707;  //
      const int acot = 16;
      float dist;
      float angle;
     static float sentido_giro;
    if(contains(recorrido)){
      std::sort( ldata.begin() , ldata.end()  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;
      dist=ldata.data()->dist;
      angle=ldata.data()->angle;
    }else{
      std::sort( ldata.begin()+ acot , ldata.end() - acot  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;
      dist=(ldata.data() + acot)->dist;
      angle=(ldata.data() + acot)->angle;
    }


     if(dist < threshold )
    {
	//state = State::SEARCH;	
	encontrado=0;
      if(angle >= 0 )
      {
	sentido_giro=-rot;
	differentialrobot_proxy->setSpeedBase(0, -rot);
      }
    
      else {
	sentido_giro=rot;
	differentialrobot_proxy->setSpeedBase(0, rot);
      }
      
      
       int  numero = rand() % 42;
       if(numero %5==0){
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
  //QMutexLocker A ;
  
 
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
	marcas.lista.push_back(t);
    }
    state = State::SEARCH;
}
  







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
  try{     
	   ldata = laser_proxy->getLaserData();
	   TBaseState tbase; differentialrobot_proxy->getBaseState(tbase);
	   inner->updateTransformValues("base",tbase.x,0,tbase.z,0,tbase.alpha,0);
	
     switch(state){

       case State::INIT:
	  std::cout << "<--------------Buscando---------->"<< std::endl;  
	        avanzar_sin_rumbo();
	      break;

        case State::SEARCH:
	         search();
	      break;

	case State::ADVANCE:
		Controller();
		
		//controller_proxy->go();
	      break;
	      
        case State::STOP:
	         differentialrobot_proxy->setSpeedBase(0,0);
	         std::cout << "<--FIN-->"<< std::endl;   
	      break;
      }
  }catch(const Ice::Exception &ex){
     std::cout << ex << std::endl;
  }	 
}



void SpecificWorker::Controller()
{

  NavState st;
  st=controller_proxy->getState();

 if(!enviado){
  enviado=true;
  std::cout << "Enviando" << std::endl;
  DatosCamara::MyTag B= marcas.get(recorrido);
  QVec realidad = inner->transform("world",QVec::vec3(B.dist_x,0,B.dist_z),"rgbd");
  
 TargetPose TP;
 TP.x=realidad.x();
 TP.z=realidad.z();
  marcas.clear();
  controller_proxy->go(TP);
}

  
  if(st.state == "FIN"){
    std::cout<<"FIN de la prueba"<<std::endl;
   state=State::SEARCH;
   enviado=false;
   recorrido++;
  }
  
  if(recorrido==4)
    state=State::STOP;
  
  
}

void SpecificWorker::search()
{
   differentialrobot_proxy->setSpeedBase(0, 0);
   DatosCamara::MyTag A;
 //  tag B;
      std::cout << "<--Buscando . . . -->"<< std::endl;   
   if(marcas.contains(recorrido)){
      //Obtenemos la marca
       A= marcas.get(recorrido);
      //Acutalizar Marca en memoria de inner
       Memoria.vec = inner->transform("world",QVec::vec3(A.dist_x,0,A.dist_z),"rgbd");
       Memoria.activo=true;
       std::cout << "<--Busqueda finalizada-->"<< std::endl;   
       state = State::ADVANCE;//Controller();
//     }else{
//       //Comprobamos que la marca este en memoria, si es asi nos dirigimos hacia ella sino nos ponemos a buscar
//        if(Memoria.activo){ 
//            QVec realidad = inner->transform("rgbd",Memoria.vec,"world");
//           //Una vez tenemos los datos reales, tenemos que hacer operaciones para orientar el robot hacia el objetivo
//            B.tx=realidad.x();
//            B.tz=realidad.z();
//            B.id=recorrido;
//            marcas.add(B);
//            state = State::ADVANCE;//Controller(); 
   }else{
	        differentialrobot_proxy->setSpeedBase(0,0.7707);
        }
    
}

void SpecificWorker::avanzar_sin_rumbo()
{

   const float threshold = 400; //Distancia previa a choquein
   float rot = 0.7707;  //
   const int acot = 16;
   float dist;
   float angle;
   static float sentido_giro;
   int  numero = rand() % 42;
   
  
   if(marcas.contains(recorrido)){
       std::sort( ldata.begin() , ldata.end()  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;
       dist=ldata.data()->dist;
       angle=ldata.data()->angle;
    }else{
       std::sort( ldata.begin()+ acot , ldata.end() - acot  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;
       dist=(ldata.data() + acot)->dist;
       angle=(ldata.data() + acot)->angle;
    }

    if(dist < threshold ){
       if(angle >= 0 ){
         sentido_giro=-rot;
        }else {
           sentido_giro=rot;
        }
        differentialrobot_proxy->setSpeedBase(0, sentido_giro);

       if(numero %5==0){
           differentialrobot_proxy->setSpeedBase(0, sentido_giro*3);
           
        }
    }else{
       differentialrobot_proxy->setSpeedBase(500, 0); 
    }
    
    state = State::SEARCH;
    
}

void SpecificWorker::evitar_obstaculos()
{
  
//    float rot = 0.7707;  //
//    float dist;
//    float angle;
//    static float sentido_giro;
//    QVec realidad = inner->transform("rgbd",Memoria.vec,"world");
//    float distancia_M=sqrt(pow(realidad.x(),2) + pow(realidad.z(),2));
//    RoboCompLaser::TLaserData ldatacopy=ldata;
//    
//    
//    
//    std::sort( ldatacopy.begin() , ldatacopy.end()  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;
//    dist=ldatacopy.data()->dist;
//    angle=ldatacopy.data()->angle;
//     
// 
//     if(dist != distancia_M )
//     {
// 
//       (ldatacopy.data()+40)->angle;
// 
//     }else{
//       	 rot=atan2(realidad.x(),realidad.z());
// 	 differentialrobot_proxy->setSpeedBase(300,rot);
//     }
}



 
////////////////////////////////////////////////////////////77
//////  EN EL HILO THE ICE
////////////////////////////////////////////////////////////

void SpecificWorker::newAprilTag(const tagsList &tags)
{
   for(auto t : tags){
	   marcas.add(t);
    }
  
}   


  

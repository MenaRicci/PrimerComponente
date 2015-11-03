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
    inner= new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}
float SpecificWorker::calcularDistancia()
{
  
  QVec realidad = inner->transform("rgbd",QVec::vec3(tag.x,0,tag.z),"world");
  float distancia=sqrt(pow(realidad.x(),2) + pow(realidad.z(),2));
  std::cout<<"La distancia es: "<< distancia <<std::endl;
  return distancia;
  
  
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
	   ldata = laser_proxy->getLaserData();
	   TBaseState tbase; differentialrobot_proxy->getBaseState(tbase);
	   inner->updateTransformValues("base",tbase.x,0,tbase.z,0,tbase.alpha,0);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Camera" << e << std::endl;
	}
	
	
  switch (state)
  {
    
       case State::IDLE:
      
      break;
      
    case State::INIT:
      
	//Reset Parametros
      
	//Pasar A control
      state=State::CONTROL;
      break;
 
    case State::CONTROL:
      if(calcularDistancia()<=10000)
	state=State::FIN;
      else
      {
	state=State::VISTA;
      }
      break;
    case State::VISTA:
     // if(isView()){
      //Avanzar hacia ojectivo
	
	state=State::ADVANCE;
	
      //}
    //  else{

	state=State::SUBOBJETIVO;
    //  }
      
      break;
      
    case State::SUBOBJETIVO:
//     if(isSubocj())
//      {
// 	if(fin_objective()){
// 	state=State::VISTA;
// 	  
// 	}
//       Avanzar hacia subojectivo
// 	
// 	state=State::ADVANCE;
//      }
//      else
//      {
//       Crear Objetivo
// 	
//      }
    break;
    case State::FIN:
      std::cout<<"Fin de la prueba de maquina de estados"<<std::endl;
      
      break;
//     
  }
	
	
	
}





////////////////////////////
//// SERVANT
/////////////////////////////

float SpecificWorker::go(const TargetPose &target)
{
  
  std::cout << "Informacion Recibida. La X vale: " << target.x << std::endl;
  tag.x=target.x;
  tag.z=target.z;

  
  state=State::INIT;
  
  
return 0.0;
}

NavState SpecificWorker::getState()
{
  NavState st;
  switch (state)
  {
    case State::INIT:
	st.state="IDLE";
      break;
    case State::FIN:
      st.state="FIN";
      break;
    default:
      st.state="WORKING";
      break;
  }
  return st;
  
}

void SpecificWorker::stop()
{

}







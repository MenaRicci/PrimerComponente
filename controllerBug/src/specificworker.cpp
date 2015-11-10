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
float SpecificWorker::calcularDistancia(float x,float z)
{
  
  QVec realidad = inner->transform("rgbd",QVec::vec3(x,0,z),"world");
  float distancia=realidad.norm2();
  
  //sqrt(pow(realidad.x(),2) + pow(realidad.z(),2));
  //std::cout<<"La distancia es: "<< distancia <<std::endl;
  return distancia;
  
  
}

bool SpecificWorker::isView()
{
  
  std::sort( ldata.begin() , ldata.end()  , [](RoboCompLaser::TData a, RoboCompLaser::TData b ){ return     a.dist < b.dist; }) ;
   float dist=ldata.data()->dist;
   std::cout<<"Estoy en view " <<std::endl;
   std::cout<<"La distancia de dist: "<< dist <<std::endl;
   std::cout<<"La distancia del tag : "<< calcularDistancia(tag.x,tag.z) <<std::endl;
   if(dist == calcularDistancia(tag.x,tag.z)) //Si estamos pegados...
     return true; 
   else
     return false;
  
}

bool SpecificWorker::isObjective()
{
return subtag.isActive;
}

bool SpecificWorker::fin_objective()
{
 //Conocer las coordenadas del robot y compararlas con el tag
  TBaseState tbase; differentialrobot_proxy->getBaseState(tbase);
	   
   if(calcularDistancia(subtag.x,subtag.z) == calcularDistancia(tbase.x,tbase.z))
	return true;    //Hemos llegado al SUBOBJETIVO
   else 
      return false;
    
}
void SpecificWorker::crearObjective()
{
  std::cout<<"Estoy en crearObjective " <<std::endl;
  uint i,j,x;
 
  for(i=ldata.size()/2; i>0; i--)
	{
		if( (ldata[i].dist - ldata[i-1].dist) < -400 )
		{
			uint k=i-2;
			while( (k >= 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < 400 ))
			{ k--; }
			x=k;
			break;
		}
	}
	for(j=ldata.size()/2; j<ldata.size()-1; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -400 )
		{
			uint k=j+2;
			while( (k < ldata.size()) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < 400 ))
			{ k++; }
			x=k;
			break;
		}
	}
  
  
  QVec subTarget_Final=inner->transform("world", QVec::vec3(ldata[x].dist *sin(ldata[x].angle)-2000,0, ldata[x].dist *cos(ldata[x].angle)), "laser");
  subtag.x=subTarget_Final.x();
  subtag.z=subTarget_Final.z();
  subtag.isActive=true;
    
  std::cout<<"Distancia X subtag " << subtag.x <<std::endl;
  std::cout<<"Distancia Z subtag " << subtag.z <<std::endl;
  
}




bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	timer.start(Period);

	return true;
}

void SpecificWorker::avanzar_subtag()
{
   
    QVec sub_data = inner->transform("laser", QVec::vec3(subtag.x,0,subtag.z), "world");
    float alpha =atan2(sub_data.x(), sub_data.z());
    float rot= 0.4*alpha;
    float dist = sub_data.norm2();
   
   
    if(dist<100)
    {
        subtag.isActive=false;

        differentialrobot_proxy->setSpeedBase(0,0);
      sleep(1);
      
    }else
    {
      if( fabs(rot) > 0.2) dist = 0;
      if(dist>300)dist=300;
      differentialrobot_proxy->setSpeedBase(dist,rot);
    }
  
   
}

void SpecificWorker::avanzar_tag()
{
   
    QVec sub_data = inner->transform("rgbd", QVec::vec3(subtag.x,0,subtag.z), "world");
    float alpha =atan2(sub_data.x(), sub_data.z());
    float rot= 0.3*alpha;
    float dist = 0.3*sub_data.norm2();
   
      if( fabs(rot) > 0.2) dist = 0;
      if(dist>300)dist=300;
      differentialrobot_proxy->setSpeedBase(dist,rot);
    
  
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
      if(calcularDistancia(tag.x,tag.z)<=300)
	state=State::FIN;
      else
      {
	state=State::VISTA;
      }
      break;
    case State::VISTA:
     if(isView()){
      //Avanzar hacia ojectivo
	state=State::ADVANCE;
      }
      else
      {
	state=State::SUBOBJETIVO;//
      }
      break;
      
    case State::SUBOBJETIVO:
      
    if(isObjective())
      {
	if(fin_objective()) //Conseguir 
	{
	  //reset del SUBOBJETIVO
	  state=State::VISTA;
 	}
 	else
	{
	  state=State::ADVANCE;
	}
      }
      else  
      {
//       Crear Objetivo --> ROTAR
	 crearObjective();
	 state=State::ADVANCE;
      }
    break;
    case State::ADVANCE:
      std::cout<<"Estoy en advance " <<std::endl;
      if(isView())
      {
      subtag.isActive=false;
      }
      if(isObjective()){
	if(fin_objective()) //Conseguir 
	  {
	    state=State::VISTA;
	    subtag.isActive=false;
	  }
	  //Avanzar hacia el SUBOBJETIVO
	  
	  avanzar_subtag();
	}
	else
	{
	    if(calcularDistancia(tag.x,tag.z)<=300)
	      state=State::FIN;
	    else{

	      avanzar_tag();
	    }
	}
      
      
      break;
    
    case State::FIN:
      std::cout<<"Fin de la prueba de maquina de estados"<<std::endl;
      
      break;	
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
      state=State::IDLE;
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







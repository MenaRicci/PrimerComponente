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

float SpecificWorker::calcularAngulo(float x,float z)
{
  
  QVec realidad = inner->transform("rgbd",QVec::vec3(x,0,z),"world");
  float angulo=atan2(realidad.x(), realidad.z() );
  
  //sqrt(pow(realidad.x(),2) + pow(realidad.z(),2));
  //std::cout<<"La distancia es: "<< distancia <<std::endl;
  return angulo;
  
  
}

bool SpecificWorker::isView()
{
  
  float d = calcularDistancia(tag.x,tag.z);
  float alpha =calcularAngulo(tag.x,tag.z);
  
  for(uint i = 0; i<ldata.size(); i++)
  {
      if(ldata[i].angle < alpha)
      {
	if( d - ldata[i].dist >10)
	{
	  return false;
	}
	else
	{
	  subtag.isActive=false;
	  return true;
	}
      }
  }
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
		if( (ldata[i].dist - ldata[i-1].dist) < -500 )
		{
		  if(i==1)
		  {
		    x=0;
		    break;
		  }
		  else
		  {
			uint k=i-2;
			while( (k >= 0) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[i-1].angle)) < 500 ))
			{ k--; }
			x=k;
			break;
		  }
		}
	}
	for(j=ldata.size()/2; j<ldata.size()-1; j++)
	{
		if( (ldata[j].dist - ldata[j+1].dist) < -500 )
		{
		    if(j==(ldata.size()-2))
		    {
		      x=j;
		      break;
		    }
		    else{
			uint k=j+2;
			while( (k < ldata.size()) and (fabs( ldata[k].dist*sin(ldata[k].angle - ldata[j+1].angle)) < 500 ))
			{ k++; }
			x=k;
			break;
		    }
		  
		}
	}
  
  
  QVec subTarget_Final=inner->transform("world", QVec::vec3(ldata[x].dist *sin(ldata[x].angle)-2000,0, ldata[x].dist *cos(ldata[x].angle)), "laser");
  subtag.x=subTarget_Final.x();
  subtag.z=subTarget_Final.z();
  subtag.isActive=true;
  
}


void SpecificWorker::crearObjectiveP()
{

  uint i, j;
  QVec t;
  float dt;
  

  t = inner->transform("rgbd", QVec::vec3(tag.x,0,tag.z), "world");

  
  float d = t.norm2();
  float alpha =atan2(t.x(), t.z() );   
    
  for(i = 5; i<ldata.size()-5; i++)
  {
      if(ldata[i].angle < alpha)
      {
	if(d>ldata[i].dist)
	{
	  dt=ldata[i].dist;
	 break;
	}
      } 
  }
  
  for(j = i;j<ldata.size()-5;j++)
  {
    
      if(ldata[j].dist> (dt+(dt*0.2)) and ldata[j].angle < 0)
      {
	QVec sub=inner->transform("world", QVec::vec3(ldata[j].dist *sin(ldata[j].angle)-2000,0, ldata[j].dist *cos(ldata[j].angle)), "laser");
	subtag.x=sub.x();
	subtag.z=sub.z();
	subtag.isActive=true;
	break;
      }
  }
  
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
    float dist = 0.3*sub_data.norm2();
   std::cout<<"La rotacion es: "<< rot <<std::endl;
   std::cout<<"La distancia es: "<< dist <<std::endl;
   
   
    if(dist<300)
    {
        subtag.isActive=false;
        differentialrobot_proxy->setSpeedBase(0,0);
      
    }else
    {
      if( fabs(rot) > 0.1)
      {
	dist = 0;
      }
      if(dist>300){dist=150;}
      differentialrobot_proxy->setSpeedBase(dist,rot);
     }
  
   
}

void SpecificWorker::avanzar_tag()
{
   
    QVec sub_data = inner->transform("rgbd", QVec::vec3(tag.x,0,tag.z), "world");
    float alpha =atan2(sub_data.x(), sub_data.z());
    float rot= 0.4*alpha;
    float dist = 0.3*sub_data.norm2();
   
      if( fabs(rot) > 0.1) dist = 0;//;else rot=0;
      if(dist>300)dist=150;
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
      
      //Rest Parametros
      state=State::VISTA;
      
      break;
    
    case State::VISTA:
      std::cout<<"----Vista----"<<std::endl;
      if(isView())
	state=State::MARCA;
      else
	state=State::SUBOBJETIVO;
	 std::cout<<"-------"<<std::endl;
      break;
    
    case State::SUBOBJETIVO:
         std::cout<<"----SUBOBJETIVO----"<<std::endl;
      if(isObjective()){  
	   std::cout<<"----EXISTE SUB-MARCA----"<<std::endl;
	if(calcularDistancia(subtag.x,subtag.z)<=400){
	  state=State::VISTA;
	  subtag.isActive=false;
	}
	else
	{
	  avanzar_subtag();
	  state=State::VISTA;
	}
	
      }
      else
      {
	std::cout<<"----CREANDO SUB-MARCA----"<<std::endl;
	crearObjectiveP();
      }
         std::cout<<"---------------"<<std::endl;
      break;
    
    case State::MARCA:
      std::cout<<"----EXISTE MARCA----"<<std::endl;
      differentialrobot_proxy->setSpeedBase(0,0);
      if(calcularDistancia(tag.x,tag.z)<=500)
	state=State::FIN;
      else
      {
	avanzar_tag();
	state=State::VISTA;
      }
      std::cout<<"---------------"<<std::endl;
      break;
    
    case State::FIN:
      std::cout<<"FIN de la marca"<<std::endl;
      break;
      
      
       
    
/*       case State::IDLE:
      
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
	  subtag.isActive=false;
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
	std::cout<<"Esta a la vista " <<std::endl;
      subtag.isActive=false;
      }
      if(isObjective()){
	if(fin_objective()) //Conseguir 
	  {
	    std::cout<<"Fin de Sub-marca " <<std::endl;
	    state=State::VISTA;
	    subtag.isActive=false;
	  }else
	  {
	  //Avanzar hacia el SUBOBJETIVO
	  std::cout<<"Avanzar hacia Sub-marca " <<std::endl;
	  avanzar_subtag();
	    
	  }
	}
	else
	{
	    if(calcularDistancia(tag.x,tag.z)<=300)
	      state=State::FIN;
	    else{
	      std::cout<<"Avanzar hacia marca " <<std::endl;
	      avanzar_tag();
	    }
	}
      
      
      break;
    
    case State::FIN:
      std::cout<<"Fin de la prueba de maquina de estados"<<std::endl;
      
      break;*/	
}


}


////////////////////////////
//// SERVANT
/////////////////////////////

float SpecificWorker::go(const TargetPose &target)
{
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







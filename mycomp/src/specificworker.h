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

/**
       \brief
       @author authorname
*/







#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#define MAXIMO 5

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList &tags);
	int obtener_dato();  
	 
   struct DatosCamara
  {
      typedef struct
      {    
	int id;
	float dist_x;
	float dist_y;
	float dist_z;
	float rot_x;
	float rot_y;
	float rot_z;
      }MyTag;
      
      std::vector<tag> lista;
      QMutex mutex;
      
      void add(tag T);
      MyTag get();
  };
  
  DatosCamara marcas;
public slots:
	void compute(); 	

private:
  int posicion=0;
  //int MAXIMO=5;
  int recorrido=0;
 // int recorrido[4]={0,0,0,0};
 void avanzar( RoboCompLaser::TLaserData copiaLaser);
 void search();
 DatosCamara::MyTag copia(tag T);
 enum class State { INIT, ADVANCE, SEARCH, STOP};
 State state = State::INIT;
	int encontrado=0;
	DatosCamara::MyTag DatoEncontrado;
	
 
};

#endif


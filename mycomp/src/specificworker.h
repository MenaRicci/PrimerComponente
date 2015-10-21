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


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void newAprilTag(const tagsList &tags);
	int obtener_dato(); 
	
	   InnerModel *inner ;
	   
	    QVec realidad;
	
	   
  struct Smemoria{
    QVec vec;
    bool activo=false;
  };
  Smemoria Memoria;
  
	   
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
      
      QList<tag> lista;
     
      MyTag get(int id);
      void add(tag T);
  };
  DatosCamara marcas;
public slots:
	void compute(); 	

private:
    void NoEncontrado(RoboCompLaser::TLaserData copiaLaser);
    void avanzar( RoboCompLaser::TLaserData copiaLaser);
    void search(RoboCompLaser::TLaserData copiaLaser);
    void dirigir_hacia_marca( RoboCompLaser::TLaserData copiaLaser);
    int contains(int id);
    
 

    QMutex mutex;
    int posicion=0;
    int recorrido=3;
    
    DatosCamara::MyTag copia(tag T);
    enum class State { INIT, ADVANCE, SEARCH, STOP};
    State state = State::INIT;
    int encontrado=0;
    DatosCamara::MyTag DatoEncontrado;
};

#endif


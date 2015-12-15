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
#include <datoscamara.h>
#include <qmat/qmat.h>

#include <lemon/list_graph.h>

#include <lemon/dijkstra.h>
#include <lemon/maps.h>

#include <iostream>

using namespace lemon;



typedef lemon::ListGraph Graph;
  typedef Graph::EdgeIt EdgeIt;
  typedef Graph::Edge Edge;
  typedef Graph::ArcIt ArcIt;
  typedef Graph::Arc Arc;
  typedef Graph::NodeIt NodeIt;
  typedef Graph::Node Node;
  typedef Graph::EdgeMap<int> LengthMap;
  using lemon::INVALID;


class SpecificWorker : public GenericWorker
{
   Q_OBJECT
   public:
	   SpecificWorker(MapPrx& mprx);	
	   ~SpecificWorker();
	   bool setParams(RoboCompCommonBehavior::ParameterList params);
	   void newAprilTag(const tagsList &tags);
	   int obtener_dato(); 
	
	   
     struct Smemoria{
       QVec vec;
       bool activo=false;
      };
      
     
   public slots:
	   void compute(); 	

   private:
    
    // InnerModelTransform *CamaraVirtual,*BaseVirtual,*April, *Robot ;
     
     
     InnerModel *inner;
     QVec realidad;
     int recorrido=0;
     enum class State { INIT, SEARCH, ADVANCE,STOP};
     State state = State::INIT;
     Smemoria Memoria;
     DatosCamara marcas;
     bool enviado=false;
     Graph graph;
     
     void Transformaciones();
     QVec getVectorAprilTag();
     void CrearCamino();
     void search();
     void Controller();;
      RoboCompLaser::TLaserData ldata;
};
#endif

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
	  std::cout << "<--------------Creando Camino---------->"<< std::endl;  
	  CrearCamino();
	  qFatal("FIN");
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


void SpecificWorker::CrearCamino()
{

    Graph g;
  
  Node s=g.addNode();
  Node v2=g.addNode();
  Node v3=g.addNode();
  Node v4=g.addNode();
  Node v5=g.addNode();
  Node t=g.addNode();

  Edge s_v2=g.addEdge(s, v2);
 // Edge v2_s=g.addEdge(v2,s);
  Edge s_v3=g.addEdge(s, v3);
  Edge v2_v4=g.addEdge(v2, v4);
  Edge v2_v5=g.addEdge(v2, v5);
  Edge v3_v5=g.addEdge(v3, v5);
  Edge v4_t=g.addEdge(v4, t);
  Edge v5_t=g.addEdge(v5, t);
    
  LengthMap length(g);

  length[s_v2]=10;
  length[s_v3]=10;
  length[v2_v4]=5;
  length[v2_v5]=8;
  length[v3_v5]=5;
  length[v4_t]=8;
  length[v5_t]=8;

  std::cout << "Hello World!" << std::endl;
  std::cout <<  std::endl;
  std::cout << "This is library LEMON here! We have a graph!" << std::endl;
  std::cout <<  std::endl;

  std::cout << "Nodes:";
  for (NodeIt i(g); i!=INVALID; ++i)
    std::cout << " " << g.id(i);
  std::cout << std::endl;

  std::cout << "Edges:";
  for (EdgeIt i(g); i!=INVALID; ++i)
    std::cout << " (" << g.id(g.u(i)) << "," << g.id(g.v(i)) << ")";
  std::cout << std::endl;
  std::cout <<  std::endl;

  std::cout << "There is a map on the edges (length)!" << std::endl;
  std::cout <<  std::endl;
  for (EdgeIt i(g); i!=INVALID; ++i)
    std::cout << "length(" << g.id(g.u(i)) << ","
              << g.id(g.v(i)) << ")="<<length[i]<<std::endl;

  std::cout << std::endl;

  
std::cout << "Dijkstra algorithm demo..." << std::endl;

    Dijkstra<Graph, LengthMap> dijkstra_test(g,length);
    
    dijkstra_test.run(v3);
    
    std::cout << "The distance of node t from node s: "
              << dijkstra_test.dist(v4) << std::endl;

    std::cout << "The shortest path from s to t goes through the following "
              << "nodes (the first one is t, the last one is s): "
              << std::endl;

    for (Node v=v4;v != v3; v=dijkstra_test.predNode(v)) {
      std::cout << g.id(v) << "<-";
    }
    
    std::cout << g.id(v3) << std::endl;  


//  ListDigraph::NodeMap<int> map(g);
    
  
  
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


  

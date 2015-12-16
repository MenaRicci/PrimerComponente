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
  
 osgView = new OsgView( frame );
 osgGA::TrackballManipulator *tb = new osgGA::TrackballManipulator;
 osg::Vec3d eye(osg::Vec3(4000.,4000.,-1000.));
 osg::Vec3d center(osg::Vec3(0.,0.,-0.));
 osg::Vec3d up(osg::Vec3(0.,-1.,0.));
 tb->setHomePosition(eye, center, up, true);
 osg::Matrixf m;
 tb->setByMatrix(osg::Matrixf::lookAt(eye,center,up));
 tb->setByMatrix(m);
 osgView->setCameraManipulator(tb);
 innerViewer = new InnerModelViewer(inner, "root", osgView->getRootGroup(), true);
 
 
TBaseState tbase; differentialrobot_proxy->getBaseState(tbase);

 QVec datos_base=QVec::vec3(tbase.x,100,tbase.z);
 
 //qDebug()<< __FUNCTION__<< "Subtarget created at " << datos_base << "with robot at" << inner->transform("world","base");
  
  
 InnerModelDraw::addPlane_ignoreExisting(innerViewer, "robotico", "world", datos_base, QVec::vec3(1,0,0), "#ff0000", QVec::vec3(400,100,400));
 
 show();
}

/**
* \brief Default destructor
*/


SpecificWorker::~SpecificWorker()
{

}
QVec SpecificWorker::getVectorAprilTag(){

    DatosCamara::MyTag A;
    QVec Vec;
   if(marcas.contains(recorrido)){
    A=marcas.get(recorrido);
    switch(A.id)
    {
      case 0:
	Vec =inner->transform("rgbd",QVec::zeros(6),"target00");
	break;
	      case 1:
	Vec =inner->transform("rgbd",QVec::zeros(6),"target01");
	break;
	      case 2:
	Vec =inner->transform("rgbd",QVec::zeros(6),"target02");
	break;
	      case 3:
	Vec =inner->transform("rgbd",QVec::zeros(6),"target03");
	break;
    }
    
   }   
   return Vec;
}

void SpecificWorker::Transformaciones()
{	
  
  QVec Vector_TargetCama=getVectorAprilTag();
  
  
RTMat RoboCama,CamaApril, Inv_RoboCama, Inv_CamaApril, WorldRobo;


inner->newTransform ("April_id", "static", inner->getNode("rgbd"), Vector_TargetCama.x(), Vector_TargetCama.y(), Vector_TargetCama.z(), Vector_TargetCama.rx(), Vector_TargetCama.ry(),Vector_TargetCama.rz(),0); //Creamos April Virtual

QVec v_1 = inner->transform("April_id",QVec::zeros(6),"rgbd" ); // Conseguimos Matriz Inversa AprilCama 



 switch(recorrido)
 {
   case 0:
    /* CamaraVirtual =inner->newTransform ("Virtual_id", "static", inner->getNode("target00"), datos_M.x(),datos_M.y() ,datos_M.z(),
					 Inv_CamaApril.getRxValue(),Inv_CamaApril.getRyValue(),Inv_CamaApril.getRzValue(), 0);
     */
   inner->newTransform ("Virtual_id", "static", inner->getNode("target00"), v_1.x(),v_1.y() ,v_1.z(),v_1.rx(),v_1.ry(),v_1.rz(), 0);
     break;
   case 1:
 inner->newTransform ("Virtual_id", "static", inner->getNode("target01"), v_1.x(),v_1.y() ,v_1.z(),v_1.rx(),v_1.ry(),v_1.rz(), 0);
     break;
     
   case 2:
 inner->newTransform ("Virtual_id", "static", inner->getNode("target02"), v_1.x(),v_1.y() ,v_1.z(),v_1.rx(),v_1.ry(),v_1.rz(), 0);
     break;
     
   case 3:
 inner->newTransform ("Virtual_id", "static", inner->getNode("target03"), v_1.x(),v_1.y() ,v_1.z(),v_1.rx(),v_1.ry(),v_1.rz(), 0);
     break;
}
//std::cout << "<--------------Inner CamaraVirtual Creado---------->"<< std::endl; 

QVec v_2 = inner->transform("rgbd",QVec::zeros(6),"base" ); 

/* InnerModelTransform* Robot =inner->newTransform ("Robot_id", "static", inner->getNode("Virtual_id"), datos_R.x(),datos_R.y() ,datos_R.z(),
					 Inv_RoboCama.getRxValue(),Inv_RoboCama.getRyValue(),Inv_RoboCama.getRzValue(), 0); //Robot Virtual Cuelga de Camara Virtual */


inner->newTransform ("Robot_id", "static", inner->getNode("Virtual_id"), v_2.x(),v_2.y() ,v_2.z(), v_2.rx(),v_2.ry(),v_2.rz(), 0);


QVec valores = inner->transform("world",QVec::zeros(6),"Robot_id"); //Transforma los valores del robot virtual al mundo
  
  
inner->updateTransformValues("base",valores.x(),valores.y(),valores.z(),valores.rx(),valores.ry(),valores.rz()); //Acutlizacion del robot 
 
TBaseState tbase; differentialrobot_proxy->getBaseState(tbase);

 QVec datos_base=QVec::vec3(tbase.correctedX,100,tbase.correctedZ);
 
 InnerModelDraw::addPlane_ignoreExisting(innerViewer, "robotico", "world", datos_base, QVec::vec3(0,1,0), "#FFAA12", QVec::vec3(400,400,100));

 inner->removeNode("Robot_id");
 inner->removeNode("Virtual_id");
 inner->removeNode("April_id");
}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{	
	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
  try{  
	
	innerViewer->update();
	osgView->autoResize();
	osgView->frame();
	

	ldata = laser_proxy->getLaserData();
	TBaseState tbase; differentialrobot_proxy->getBaseState(tbase);
	inner->updateTransformValues("base",tbase.x,0,tbase.z,0,tbase.alpha,0);
  
	 //  QVec datos_base=QVec::vec3(tbase.x,100,tbase.z);
	// InnerModelDraw::addPlane_ignoreExisting(innerViewer, "robotico", "world", datos_base, QVec::vec3(1,0,0), "#ff0000", QVec::vec3(400,100,400));
  
     switch(state){

       case State::INIT:
	  //std::cout << "<--------------Creando Camino---------->"<< std::endl;  
	
	
	  Transformaciones();
	
	
	
	      break;

        case State::SEARCH:
	 
	         search();
		 /*Cuando Encuentre una marca, el robot se va a autoajustar para que se mueva de manera correcta*/
		 
		 
	//	 differentialrobot_proxy->setOdometerPose(tbase.correctedX,tbase.correctedZ,tbase.correctedAlpha);
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

  std::cout << "<--------------En controller---------->"<< std::endl; 
  
 if(!enviado){
 enviado=true;
 std::cout << "Enviando" << std::endl;
 TBaseState tbase; differentialrobot_proxy->getBaseState(tbase);
 
 
 DatosCamara::MyTag B= marcas.get(recorrido);

 
 QVec realidad = inner->transform("world",QVec::vec3(B.dist_x,0,B.dist_z),"rgbd"); 
 
  marcas.clear();

   TargetPose TP;
   TP.x=Memoria.vec.x();
   TP.z=Memoria.vec.z();

  
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
    //  std::cout << "<--Buscando . . . -->"<< std::endl;   
   if(marcas.contains(recorrido)){
      //Obtenemos la marca
       A= marcas.get(recorrido);
      //Acutalizar Marca en memoria de inner
       Memoria.vec = inner->transform("world",QVec::vec3(A.dist_x,0,A.dist_z),"rgbd");
       Memoria.activo=true;
       std::cout << "<--Busqueda finalizada-->"<< std::endl; 
       
       
       state = State::ADVANCE; 
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


  

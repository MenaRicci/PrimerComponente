/*
 * Copyright 2015 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "datoscamara.h"

DatosCamara::DatosCamara()
{

}


DatosCamara::~DatosCamara()
{

}

DatosCamara::MyTag DatosCamara::get(int id)
{
   DatosCamara::MyTag A;
   QMutexLocker M(&mutex);
   for(auto T : lista){
     if(T.id==id){   
        A.id=T.id;
        A.dist_x=1000 * T.tx;
        A.dist_y=1000 * T.ty;
        A.dist_z=1000 * T.tz;
        A.rot_x=T.rx;
        A.rot_y=T.ry;
        A.rot_z=T.rz;

        return A;
      }
    }
   return A;
}
    
int DatosCamara::contains(int id)
{
   QMutexLocker M(&mutex);
   for(auto t : lista){
     if(t.id==id)
        return 1;
    }
   return 0;
}

void DatosCamara::add(tag T)
{
   QMutexLocker M(&mutex);
   lista.push_back(T);
}

void DatosCamara::clear()
{
  QMutexLocker M(&mutex);
   lista.clear();
}





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

#ifndef DATOSCAMARA_H
#define DATOSCAMARA_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

class DatosCamara
{
  
  private:
       QList<tag> lista;
       QMutex mutex;

 public:
       typedef struct{    
         int id;
         float dist_x;
	       float dist_y;
	       float dist_z;
	       float rot_x;
	       float rot_y;
	       float rot_z;
        }MyTag;
   		
int contains(int id);
MyTag get(int id);
void add(tag T);
void clear();
  
DatosCamara();
DatosCamara(const DatosCamara& other);
~DatosCamara();
DatosCamara& operator=(const DatosCamara& other);
bool operator==(const DatosCamara& other) const;
};

#endif // DATOSCAMARA_H




#include "Anchor.h"

namespace ORB_SLAM3
{
    
std::vector<Eigen::Vector3f> Anchor::getRefPositions(){
    vector<MapPoint*> vMp=this->refs;
    std::vector<Eigen::Vector3f> vPos;
    for(MapPoint* mp:vMp){
        Eigen::Vector3f t1=mp->GetWorldPos();
       // Eigen::Vector3f t2(t1[0],t1[1],t1[2]);
        vPos.push_back(t1);
    }

    return vPos;

}

Eigen::Vector3f Anchor::update(long unsigned int mapId,std::vector<Eigen::Vector3f> prevPos){



if(mapId!=this->map->GetId())
return this->pos;

if(prevPos.size()!=this->refs.size())
std::cerr<<"No. of refs varies"<<endl;
int i=0;
double wi=1,swi=0;
Eigen::Vector3f c(0,0,0);
for(Eigen::Vector3f pp:prevPos){
    wi=1/(pp-this->pos).norm();
    Eigen::Vector3f cp=this->refs.at(i)->GetWorldPos();
    Eigen::Vector3f ci=pp-this->pos-cp;
    c=c+(wi*ci);
    swi=swi+wi;
    i++;
}
this->pos=-1*c/swi;



cout<<"Anchor pos="<<this->pos[0]<<","<<this->pos[1]<<","<<this->pos[2]<<std::endl;
 i=0;
for(Eigen::Vector3f pp:prevPos){

    Eigen::Vector3f cp=this->refs.at(i)->GetWorldPos();
    
    std::cout<<"Prev pos="<<pp[0]<<","<<pp[1]<<","<<pp[2]<<" Current POs="<<cp[0]<<","<<cp[1]<<","<<cp[2]<<std::endl;
    i++;
}


return this->pos;
}


}
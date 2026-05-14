#ifndef OPERATESTEP_H
#define OPERATESTEP_H
#include<pcl/point_types.h>
//#include<QVector>
template <typename T>
class operateStep
{
public:
    //operateStep<T>::operateStep() {}
    operateStep(){

    }
    //对于点云的一次修改，通常修改位置和颜色,可能需要修改多个点
     //<index_t,pair<pre,new>>?pair<pre,new>？pair<index_t,pre>是否需要记录被编辑点在点云的序号,yes
    std::vector<std::pair<pcl::index_t,T>> operates;

};

#endif // OPERATESTEP_H

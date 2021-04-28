//
// Created by renjie on 2020/11/30.
//

#ifndef ROAD_WIDTH_V2_CONTAINERPTR_H
#define ROAD_WIDTH_V2_CONTAINERPTR_H

#include "../common_include.h"

template <class T>
class DataPtrContainer{
private:
    T dataPtr_;
    mutex mlock_;
    chrono::system_clock::time_point t;

public:
    DataPtrContainer()= default;
    void init(const T& ptr);
    void upDate( T data);
    T getDataPtr();
    chrono::system_clock::time_point getTimePoint();

};

template <class T>
void DataPtrContainer<T>::init(const T& ptr){
    lock_guard<mutex>  lg(mlock_);
    dataPtr_=ptr;
}


template <class T>
void DataPtrContainer<T>::upDate( T dataPtr) {
    lock_guard<mutex>  lg(mlock_);
    *dataPtr_=*dataPtr;
    t=chrono::system_clock::now();
}


template <class T>
T DataPtrContainer<T>::getDataPtr() {
    lock_guard<mutex>  lg(mlock_);
    return dataPtr_;
}

template<class T>
chrono::system_clock::time_point DataPtrContainer<T>::getTimePoint(){
    return t;
}
#endif //ROAD_WIDTH_V2_CONTAINERPTR_H

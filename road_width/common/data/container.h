//
// Created by renjie on 2020/11/27.
//

#ifndef ROAD_WIDTH_V2_CONTAINER_H
#define ROAD_WIDTH_V2_CONTAINER_H

#include "../common_include.h"

template <class T>
class DataContainer{
private:

    T data_;
    bool new_;
    mutex mlock_;
    int id_;
    chrono::system_clock::time_point t;

public:
    DataContainer();
    bool isNew();
    void upDate(const T& data);
    T getData();
    T* getDataAddr();
    int getID();
    chrono::system_clock::time_point getTimePoint();
};

template<class T>
DataContainer<T>::DataContainer():new_(false),id_(0){}

template <class T>
bool DataContainer<T>::isNew(){
    lock_guard<mutex> lg(mlock_);
    return new_;
}

template <class T>
void DataContainer<T>::upDate(const T& data) {
    lock_guard<mutex>  lg(mlock_);
    id_++;
    data_=data;
    new_=true;
    t=chrono::system_clock::now();
}

template <class T>
T DataContainer<T>::getData() {
    lock_guard<mutex>  lg(mlock_);
    new_= false;
    return data_;
}

template <class T>
int DataContainer<T>::getID() {
    lock_guard<mutex>  lg(mlock_);
    return id_;
}

template <class T>
T* DataContainer<T>::getDataAddr() {
    lock_guard<mutex>  lg(mlock_);
    return &data_;
}

template<class T>
chrono::system_clock::time_point DataContainer<T>::getTimePoint(){
    return t;
}

#endif //ROAD_WIDTH_V2_CONTAINER_H

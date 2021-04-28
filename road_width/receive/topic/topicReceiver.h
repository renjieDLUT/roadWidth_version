//
// Created by renjie on 2021/1/7.
//

#ifndef FOD_TOPICRECEIVE_H
#define FOD_TOPICRECEIVE_H

#include"common_include.h"
#include"baseReceiver.h"


template<typename T>
class TopicReceiver : public BaseReceiver{
private:
    DataContainer<T>* dataPtr_;
    zros::core::Subscriber::SharedPtr subscriber_;
    string topicName_;
public:
    TopicReceiver(string topicName);
    virtual void setDataContainerPtr( void* ptr) override;
    virtual void start() override;
    virtual void stop() override;
};

template<typename T>
TopicReceiver<T>::TopicReceiver(string topicName) : topicName_(topicName), subscriber_( zros::core::Transport::Instance()->CreateSubscriber(topicName) ){
    subscriber_->RegisterCallback<T>([this](const std::string& topic,const  T& msg )
                                     {
                                         dataPtr_->upDate(msg);
//                                         cout<<"receive topicMsg:"<< topic<<endl;
                                     });
}



template<typename T>
void TopicReceiver<T>::setDataContainerPtr( void* ptr){
    dataPtr_ = (DataContainer<T>*)ptr;
}
template<typename T>
void TopicReceiver<T>::start(){
    subscriber_->Start();
}

template<typename T>
void TopicReceiver<T>::stop(){
    if (subscriber_.get()) {
        subscriber_->Stop();
        //subscriber_.reset();
    }
}


#endif //FOD_TOPICRECEIVE_H

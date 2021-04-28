//
// Created by renjie on 2021/1/8.
//

#ifndef FOD_LIVEMSGRECEIVE_H
#define FOD_LIVEMSGRECEIVE_H

#include"baseReceiver.h"
#include"common_include.h"

template<typename T>
class LiveMsgReceiver: public BaseReceiver {
private:
    void SocMsgHandler(const struct LIVE_MSG*);
    zros::core::Subscriber::SharedPtr soc_subscriber_;  // for mcu to soc
    DataContainer<T>* dataPtr_;
    int liveMsgCategoryNameId_ ;
    live_msg_id_t liveMsgId_;

public:
    LiveMsgReceiver(int liveMsgCategoryNameId, live_msg_id_t liveMsgId) ;
    ~LiveMsgReceiver() = default;
    virtual void setDataContainerPtr( void* ptr);
    virtual void start();
    virtual void stop();
};

template<typename T>
LiveMsgReceiver<T>::LiveMsgReceiver(int liveMsgCategoryNameId , live_msg_id_t liveMsgId):liveMsgCategoryNameId_(liveMsgCategoryNameId),liveMsgId_(liveMsgId){
}

template<typename T>
void  LiveMsgReceiver<T>::setDataContainerPtr(void* ptr) {
    dataPtr_ = (DataContainer<T>*)ptr;
}

template<typename T>
void LiveMsgReceiver<T>::start() {
    if (!soc_subscriber_.get()) {
        soc_subscriber_ = zros::core::Transport::Instance()->CreateSubscriber(
                live_msg_category_name(liveMsgCategoryNameId_));
        ZASSERT(soc_subscriber_);

        soc_subscriber_->RegisterCallback<zros::core::PlainBufMessage>(
                [this](const std::string &, const zros::core::PlainBufMessage &msg) {
                    SocMsgHandler((struct LIVE_MSG *) msg.GetData());
                });
    }

    soc_subscriber_->Start();
}

template<typename T>
void LiveMsgReceiver<T>::SocMsgHandler(const struct LIVE_MSG* msg){

    if (msg && msg->id == liveMsgId_ &&
        msg->data_size == sizeof(T)) {
        dataPtr_->upDate( *( (T*)msg -> data ) );
        cout<<"********received liveMsg********: "<<liveMsgId_<<endl;
    }
//    cout<<"liveMsg socMsgHnadler: "<<msg->id<<endl;

}


template<typename T>
void LiveMsgReceiver<T>::stop() {
    if (soc_subscriber_.get()) {
        soc_subscriber_->Stop();
        //soc_subscriber_.reset();
    }
}



#endif //FOD_LIVEMSGRECEIVE_H

//
// Created by renjie on 2020/11/27.
//

#include"config.h"

namespace ca{
    Config::~Config() {
        if(file_.isOpened()){
            file_.release();
        }
    }
    void Config::setParameterFile(const string &filename) {
        if (config_== nullptr){
            config_=make_shared<Config>(Config());
        }
        config_->file_=cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
        if(config_->file_.isOpened()==false){
            cerr<<"parameter file "<<filename<<"does not exist."<<endl;
            config_->file_.release();
            exit(0);
        }
    }
    shared_ptr<Config> Config::config_= nullptr;
}
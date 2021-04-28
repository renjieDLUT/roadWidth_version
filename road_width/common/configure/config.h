//
// Created by renjie on 2020/11/27.
//

#ifndef ROAD_WIDTH_V2_CONFIG_H
#define ROAD_WIDTH_V2_CONFIG_H

#include "common_include.h"

namespace  ca{
    class Config{
    private:
        static std::shared_ptr<Config> config_;
        Config()= default;
        cv::FileStorage file_;
    public:
        ~Config();
        static void setParameterFile(const string& filename);

        template <typename  T>
        static T get(const string& key){
            return T(config_->file_[key]);
        }
    };
}



#endif //ROAD_WIDTH_V2_CONFIG_H

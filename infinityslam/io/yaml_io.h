/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_IO_YAML_IO_H_
#define INFINITYSLAM_IO_YAML_IO_H_

#include <string>
#include <cassert>
#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

namespace infinityslam {

namespace io {

class YamlNode {
 public:
    YamlNode() {}
    YamlNode(const std::string& yaml_file) {
        yaml_node_ = YAML::Load(yaml_file);
        assert(!yaml_node_.IsNull());
        assert(yaml_node_.Type() == YAML::NodeType::Map);
        valid_ = true;
    }

    ~YamlNode() {}

    bool is_valid() {
        return valid_;
    }

    // 如果解析到了相应key，则改写value；否则不改变value。
    template<typename T>
    bool GetValue(const std::string& key, T& value, bool log = false) {
        if (yaml_node_[key]) {
            value = yaml_node_[key].as<T>();
            if (log) {
                LOG(INFO) << "Loaded parameter " 
                    << key << ": " << value;
            }
            return true;
        }
        if (log) {
            LOG(INFO) << "Loaded parameter " 
                << key << ": found no key, failed!";
        }
        return false;
    }

    // 如果解析到了相应key，则返回相应值；否则返回类型默认值。
    template<typename T>
    T GetValue(const std::string& key) {
        //
        return yaml_node_[key].as<T>();
        // return T();
    }

    template<typename T>
    bool SetValue(const std::string& key, const T& value) {
        // TODO
        // yaml_node_[key].push_back(T);
        return false;
    }

    bool LoadFile(const std::string& yaml_file) {
        yaml_node_ = YAML::Load(yaml_file);
        assert(yaml_node_.Type() != YAML::NodeType::Null);
        return true;
    }

    bool Save(const std::string& yaml_file) {
        // TODO
        // 
        return false;
    }



 private:
    YAML::Node yaml_node_;
    bool valid_ = false;

};

bool ConvertXXXToYaml(const std::string& xxx_file);
bool ConvertYamlToXXX(const std::string& yaml_file);


} // namespace io

} // namespace infinityslam


#endif // INFINITYSLAM_IO_YAML_IO_H_
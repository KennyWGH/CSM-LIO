/**
 * Copyright 2022 WANG Guanhua (wangxxx@gmail.com)
*/

#ifndef INFINITYSLAM_COMMON_OPTIONAL_H_
#define INFINITYSLAM_COMMON_OPTIONAL_H_

// #include

namespace infinityslam {
namespace common {


/**
 * 自己实现一个简单的 optional<> 模板类，用于替代 abSL::optional 和 C++ std::optional；
 * 鉴于后者仅在C++17标准后出现，我们必须自己实现这个功能。
 * 注意，模板数据类型必须可以自动析构 —— 因为模板类不会在析构函数中手动析构数据，
 * 这意味着本模板不支持裸指针数据类型。
 * 重载了“=, ==, +, -”共4个运算符。
*/
template<typename TValueType>
class optional {
 public:
    optional(const TValueType& rvalue) 
        :data_(rvalue), hasvalue_(true) {}
    optional() {}
    ~optional() {}

    // // 需要支持拷贝构造
    // optional(const optional& obj)
    //     :data_(obj.value()), hasvalue_(obj.has_value()) {}

    bool has_value() const {return hasvalue_;}

    const TValueType& value() const {return data_;}

    // 必须在确保赋值的情况下方可调用。
    TValueType* mutable_value() {
        if (hasvalue_) return &data_;
        return nullptr; //需要时再完善这个handle策略。
    }

    void operator=(const TValueType& rvalue) {
        data_ = rvalue;
        hasvalue_ = true;
    }

    bool operator==(const TValueType& rvalue) const {
        if (!hasvalue_) return false;
        return data_ == rvalue;
    }

    bool operator>=(const TValueType& rvalue) const {
        if (!hasvalue_) return false;
        return data_ >= rvalue;
    }

    bool operator>(const TValueType& rvalue) const {
        if (!hasvalue_) return false;
        return data_ > rvalue;
    }

    bool operator<(const TValueType& rvalue) const {
        if (!hasvalue_) return false;
        return data_ < rvalue;
    }

    TValueType operator+(const TValueType& rvalue) const {
        return data_ + rvalue;
    }

    TValueType operator-(const TValueType& rvalue) const {
        return data_ - rvalue;
    }

 private:
    TValueType data_;
    bool hasvalue_ = false;
};


}  // namespace common
}  // namespace infinityslam

#endif  // INFINITYSLAM_COMMON_OPTIONAL_H_
/**
 * Copyright 2022 WANG_Guanhua(wangxxx@gmail.com)
 * Copyright 2016 The Cartographer Authors
 * Licensed under the Apache License, Version 2.0 (the "License").
*/

#ifndef INFINITYSLAM_COMMON_INCREMENTAL_GRID_H_
#define INFINITYSLAM_COMMON_INCREMENTAL_GRID_H_


#include <cmath>
#include <cstddef>
#include <cassert>
#include <limits>
#include <array>
#include <vector>
#include <unordered_map>
#include <list>

#include <boost/make_unique.hpp>
#include <boost/format.hpp>
#include "Eigen/Core"
#include <glog/logging.h>

namespace infinityslam {
namespace common {

namespace {

/// 把一个每一维的值都位于 [0, 2^bits-1] 区间的三维索引转化为 Z-Major 的一维索引。
inline int ToFlatIndex(const Eigen::Array3i& index, const int bits) 
{
    assert((index >= 0).all() && (index < (1 << bits)).all());
    return (((index.z() << bits) + index.y()) << bits) + index.x();
}

/// 把一个 Z-Major 的一维索引转化为三维索引，三维索引的每一维度的值位于 [0, 2^bits-1] 区间。
inline Eigen::Array3i To3DIndex(const int index, const int bits) 
{
    assert(index < (1 << (3 * bits)));
    const int mask = (1 << bits) - 1;
    return Eigen::Array3i(index & mask, (index >> bits) & mask,
                            (index >> bits) >> bits);
}

/// 检查数据类型对象的值是否为默认构造值。(Allows specializations)
template <typename TValueType>
bool IsDefaultValue(const TValueType& v) 
{
    return v == TValueType();
}

/// 检查一个容器对象是否为默认构造对象（实质是检查是否为空）。
template <typename TElementType>
bool IsDefaultValue(const std::vector<TElementType>& v) 
{
    return v.empty();
}

/// 判断两个三维索引是否相等。
bool IsEqualIndex(const Eigen::Array3i& a, const Eigen::Array3i& b) 
{
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

} // namespace

/// 在一个连续的内存空间（数组）上存储 2^kBits * 2^kBits * 2^kBits 个 cell，cell 的
/// 类型为模板类型<TValueType>；每个维度上的索引计数都从0开始，到 2^kBits-1 为止。
/// 取名为“cube”，有“魔方”之意，正如一个魔方有很多个小立方块组成一样。
template <typename TValueType, int kBits>
class CubeT {
 public:
    using CellType = TValueType;

    // 默认构造函数：按模板数据类型的默认构造函数初始化所有模板数据类型对象。
    CubeT() 
    {
        for (CellType& value : cells_) {
        value = CellType();
        }
    }

    CubeT(const CubeT&) = delete;
    CubeT& operator=(const CubeT&) = delete;

    // 【类静态函数】返回 cube 的一个维度上有多少个 cell。
    static int cube_length() { return 1 << kBits; }

    // 返回三维索引位置上的 cell 数据；
    // 注意三维索引每一维的取值必须位于 [0, cube_length()-1] 区间。
    CellType value(const Eigen::Array3i& index) const 
    { return cells_[ToFlatIndex(index, kBits)]; }

    // 返回三维索引位置上的 cell 数据的指针，以便允许修改它的值。
    CellType* mutable_value(const Eigen::Array3i& index) 
    { return &cells_[ToFlatIndex(index, kBits)]; }

    // wgh:基于指针实现一个迭代器，用于访问所有非默认值的cell（大部分迭代器的底层本来就是指针）。
    class Iterator {
      public:
        Iterator() : current_(nullptr), end_(nullptr) {}

        // 该构造函数将current指向第一个非默认值cell。
        explicit Iterator(const CubeT& flat_cube)
            : current_(flat_cube.cells_.data()),
            end_(flat_cube.cells_.data() + flat_cube.cells_.size()) 
        {
            while (!Done() && IsDefaultValue(*current_)) {
                ++current_;
            }
        }

        // 遍历到下一个非默认值cell。
        void Next() {
            assert(!Done());
            do {
                ++current_;
            } while (!Done() && IsDefaultValue(*current_));
        }

        bool Done() const { return current_ == end_; }

        // 获得当前指向cell的三维索引。
        Eigen::Array3i GetCellIndex() const {
            assert(!Done());
            const int index = (1 << (3 * kBits)) - (end_ - current_);
            return To3DIndex(index, kBits);
        }

        // 获得当前指向cell的值。
        const CellType& GetValue() const {
            assert(!Done());
            return *current_;
        }

      private:
        const CellType* current_;
        const CellType* end_;
    };

 private:
    std::array<CellType, 1 << (3 * kBits)> cells_;
};

/// 定义空间哈希函数，作为[参数]用于定义空间哈希表。
struct SpaceHashFunc {
    inline size_t operator()(const Eigen::Array3i& key) const
    {
        return size_t(((key[0]) * 73856093) ^ ((key[1]) * 471943) ^ ((key[2]) * 83492791)) 
            % 10000000;
    }
};

/// 定义"=="判断函数，作为[参数]用于定义空间哈希表。
struct IsEqualFunc {
    inline bool operator()(const Eigen::Array3i& a, const Eigen::Array3i& b) const
    {
        return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
    }
};

/// 定义“<”判断函数，作为[参数]用于定义“严格弱序”类容器，比如std::set。
struct IsLessFunc {
    bool operator() (const Eigen::Array3i& lkey, const Eigen::Array3i& rkey) const
    {
        // // return lkey < rkey;
        // return (size_t(((lkey[0]) * 73856093) ^ ((lkey[1]) * 471943) ^ ((lkey[2]) * 83492791))  % 10000000)
        //     < (size_t(((rkey[0]) * 73856093) ^ ((rkey[1]) * 471943) ^ ((rkey[2]) * 83492791))  % 10000000);

        // Z-Major Ordering
        return lkey[2] < rkey[2] ? true 
                    : lkey[2] > rkey[2] ? false
                        :lkey[1] < rkey[1] ? true 
                            : lkey[1] > rkey[1] ? false
                                :lkey[0] < rkey[0] ? true 
                                    : false;
    }
};


/// 简介：基础的增量式数据接口，通过哈希表持有离散的cube，cube持有固定数量的连续的cell。
/// Internally, IncrementalCube uses a hash table to dynamically maintain all 'Cube'.
/// 'Cube' are constructed on first access via 'mutable_value()'. If necessary,
/// the 'Cube' grows at the position that is reached.
/// 补充：定义空间的三级划分 —— Cell 构成 Cube 构成 IncrementalCube(s)。
/// Note：外部无需知道cube的存在，只需要按cell来调用接口即可。
template <typename CellType, int kBits>
class IncrementalCube {

    /** ### readme ###
     * When kBits = 5, a cube is of 32*32*32 cells;    if cell_size=0.05, then cubic is 1.6*1.6*1.6    m3.
     * When kBits = 5, a cube is of 32*32*32 cells;    if cell_size=0.1,  then cubic is 3.2*3.2*3.2    m3.
     * When kBits = 6, a cube is of 64*64*64 cells;    if cell_size=0.1,  then cubic is 6.4*6.4*6.4    m3.
     * When kBits = 7, a cube is of 128*128*128 cells; if cell_size=0.1,  then cubic is 12.8*12.8*12.8 m3.
     * When kBits = 8, a cube is of 256*256*256 cells; if cell_size=0.1,  then cubic is 25.6*25.6*25.6 m3.
     * Note that we have to carefull manage the number of cubes in order to control memory usage!
     * For the best, we wish the memory usage is always less then 4 GB! 
     * The CellType(i.e., cell) we use is of two int type variables, that is 8 byte.
     * To achieve this, the number of cells in total should be less than 5.3e8;
     * In fact, cube number 1100*1100*400=4.84e8 meets this requirement.
     * In this case, with cell_size=0.05m, we have a 3D space of 55*55*20 m3.
     * Also, with cell_size=0.1m, we have a 3D space of 110*110*40 m3.
     * Index Definition【索引定义】: 
     * Note that the CUBE with index [0,0] is TOTALLY in first quadrant, 
     * while [-1,-1] is TOTALLY in third quadrant.
     *           |        
     *           |--------|   
     *           | [0,0]  |    
     *  ---------|--------------
     *    [-1,-1]|        
     *           |  
     *           |              
    */

  public:
    using CubeImpl = CubeT<CellType, kBits>; 
    // using CellType = typename CubeImpl::CellType;

    // 带默认参数的构造函数。
    IncrementalCube(const float& cell_size = 0.1, 
                    const float& max_range = 60.0) 
        : kCellSize(cell_size)
        , kCellSizeInverse(1.f/cell_size)
        , kMaxRange(max_range)
    {
        LOG(INFO) << "******************************************** ";
        LOG(INFO) << "*** Constructed IncrementalCube object with:";
        LOG(INFO) << "*** kCellSize = " << kCellSize;
        LOG(INFO) << "*** kMaxRange = " << kMaxRange;
        LOG(INFO) << "*** kBits = " << kBits;

        // 作为初始化，以原点为中心构造一个【40米*40米*20米】的 cube map。 
        const int num_cube_length = 
            40.f / kCellSize / GetCubeLength() + 1;
        const int num_cube_height =  
            20.f / kCellSize / GetCubeLength() + 1;
        const Eigen::Array3i offset = 
        Eigen::Array3i(num_cube_length / 2, num_cube_length / 2, num_cube_height / 2);
        size_t num_cubes = 0;
        const size_t total = (num_cube_length) * (num_cube_length) * (num_cube_height);
        for (int i = 0; i < num_cube_length; ++i) {
            for (int j = 0; j < num_cube_length; ++j) {
                for (int k = 0; k < num_cube_height; ++k) {
                    Eigen::Array3i cube_index = Eigen::Array3i(i,j,k) - offset;
                    cubes_list_.push_front(
                        std::make_pair(cube_index, 
                            boost::make_unique<CubeImpl>()));
                    cubes_map_.insert(std::make_pair(cube_index, cubes_list_.begin()));
                    ++num_cubes;
                }
            }
        }
        OBB_out_of_date_ = true;

        LOG(INFO) << "*** cube length = " << (1 << kBits);
        LOG(INFO) << "*** cube size (m) = " << (1 << kBits) * kCellSize;
        LOG(INFO) << "*** initialized with cube number = " << cubes_list_.size();
        LOG(INFO) << "******************************************** ";

        UpdateObbBoundary();
    } 

    IncrementalCube(IncrementalCube&&) = default;
    IncrementalCube& operator=(IncrementalCube&&) = default;

    // 返回当前维护的Cube的总数。
    int GetNumOfCubes() const { return cubes_list_.size(); }

    // 返回当前维护的Cell的总数。
    int GetNumOfCells() const { return cubes_list_.size() * (1 << (3 * kBits)); }

    // 返回一个Cube的一个维度上的Cell的数量。
    int GetCubeLength() const { return CubeImpl::cube_length(); } 

    // 返回cell的边长（米）。
    float GetCubeSize() const { return CubeImpl::cube_length() * kCellSize; }

    // 返回cell的边长（米）。
    float GetCellSize() const { return kCellSize; }

    // 通过 point 的世界系三维坐标，查找所在 cell 的世界系三维索引。
    Eigen::Array3i PositionToCellIndex(const Eigen::Vector3f& point) const {
        Eigen::Array3f index = point.array() * kCellSizeInverse;
        // return Eigen::Array3i(std::lround(index.x()), // 四舍五入为round.
        //                     std::lround(index.y()),
        //                     std::lround(index.z()));

        // wgh:难道不应该是floor吗?(floor版本)
        return Eigen::Array3i(std::floor(index.x()),
                              std::floor(index.y()),
                              std::floor(index.z()));
    }

    // 返回cell的值。
    CellType value(const Eigen::Array3i& index) const 
    {
        const Eigen::Array3i CUBE_index = CellIndexToCubeIndex(index);
        const Eigen::Array3i inner_index =
            index - CUBE_index * CubeImpl::cube_length();
        // Handle exception.
        // assert((inner_index >= 0).all() && (inner_index < (1 << kBits)).all());
        if ((inner_index < 0).any() || (inner_index >= (1 << kBits)).any()) {
            LOG(ERROR) << "Query cell index [" << index.transpose() 
                << "] leading to cube index [" << CUBE_index.transpose() 
                << "] and inner index [" << inner_index.transpose() 
                << "] -- but inner index overflows!";
            return CellType();
        }
        auto map_ite = cubes_map_.find(CUBE_index);
        if (map_ite != cubes_map_.end()) {
            // Handle exception.
            // assert(IsEqualIndex(CUBE_index, map_ite->second->first));
            if (!IsEqualIndex(CUBE_index, map_ite->second->first)) {
                LOG(ERROR) << "Deduced cube index [" << CUBE_index
                    << "] conflicts with registered cube index ["
                    << map_ite->second->first << "], fatal error!";
                return CellType();
            }
            // Query as expected.
            std::unique_ptr<CubeImpl>& CUBE_ptr_ = map_ite->second->second;
            return CUBE_ptr_->value(inner_index);
        } else {
            LOG(WARNING) << "Query cell index [" << index.transpose() 
                << "] leading to cube index [" << CUBE_index.transpose() 
                << "] and inner index [" << inner_index.transpose() 
                << "] -- but this cube doesn't exist yet!";
            return CellType();
        }

        // Handle exception.
        LOG(ERROR) << "Query cell index [" << index.transpose() 
            << "] leading to cube index [" << CUBE_index.transpose() 
            << "] and inner index [" << inner_index.transpose() 
            << "] -- not supposed to reach here, fatal error!";
        return CellType();
    }

    // 返回指向cell的指针，以便读写；若cell位置不存在，则创建新Cube。
    CellType* mutable_value(const Eigen::Array3i& index) 
    {
        Eigen::Array3i CUBE_index = CellIndexToCubeIndex(index);
        const Eigen::Array3i inner_index =
            index - CUBE_index * CubeImpl::cube_length();
        // Handle exception.
        // assert((inner_index >= 0).all() && (inner_index < (1 << kBits)).all());
        if ((inner_index < 0).any() || (inner_index >= (1 << kBits)).any()) {
            LOG(ERROR) << "Query cell index [" << index.transpose() 
                << "] leading to cube index [" << CUBE_index.transpose() 
                << "] and inner index [" << inner_index.transpose() 
                << "] -- but inner index overflows!";
            return &exception_cell;
        }

        auto map_ite = cubes_map_.find(CUBE_index);
        if (map_ite != cubes_map_.end()) {
            // Handle exception.
            assert(IsEqualIndex(CUBE_index, map_ite->second->first));
            if (!IsEqualIndex(CUBE_index, map_ite->second->first)) 
            {
                LOG(ERROR) << "Deduced cube index [" << CUBE_index
                    << "] conflicts with registered cube index ["
                    << map_ite->second->first << "], fatal error!";
                return &exception_cell;
            }
            // Query as expected.
            std::unique_ptr<CubeImpl>& CUBE_ptr_ = map_ite->second->second;
            return CUBE_ptr_->mutable_value(inner_index);
        }
        else {
            // 需要新增CubeImpl对象.
            cubes_list_.push_front(
                std::make_pair(CUBE_index, boost::make_unique<CubeImpl>()));
            cubes_map_.insert(std::make_pair(CUBE_index, cubes_list_.begin()));
            OBB_out_of_date_ = true;
            std::unique_ptr<CubeImpl>& CUBE_ptr_ = cubes_list_.front().second;
            return CUBE_ptr_->mutable_value(inner_index);
        }

        // Handle exception.
        LOG(ERROR) << "Query cell index [" << index.transpose() 
            << "] leading to cube index [" << CUBE_index.transpose() 
            << "] and inner index [" << inner_index.transpose() 
            << "] -- not supposed to reach here, fatal error!";
        return &exception_cell;
    }

    // 检查和删除超出2倍MaxRange范围的CubeImpl对象. (TODO here)
    void TrimAroundMovingCenter(const Eigen::Vector3f& center)
    {
        // 找出当前center所在CUBE坐标；
        // 根据 kMaxRange 找出容许保存的范围;
        // 如果，链表容器中任何CUBE的任何一维超出容许范围，则删除之。
        curr_center_ = center;
        Eigen::Array3i center_CUBE_index = 
            CellIndexToCubeIndex(PositionToCellIndex(center));
        const int max_distance_to_keep = 
            kMaxRange * kCellSizeInverse / CubeImpl::cube_length();
        for (auto list_ite = cubes_list_.begin(); list_ite != cubes_list_.end(); ) {
            // 如果当前CUBE到center_CUBE的距离超过设定阈值,则删除之.
            Eigen::Array3i distance = (list_ite->first - center_CUBE_index).abs();
            if ((distance > max_distance_to_keep).any()) {
                cubes_map_.erase(list_ite->first);
                list_ite = cubes_list_.erase(list_ite);
                // 执行list::erase()后,迭代器自动指向下一个位置,无需自加.
            }
            else {
                // 迭代器自加.
                ++list_ite;
            }
        }
        return;
    } 

    Eigen::Vector3f current_center() { return curr_center_; }

    // 遍历所有cube以更新包络盒的边界信息.
    void UpdateObbBoundary() {
        const int MIN_VALUE = std::numeric_limits<int>::min();
        const int MAX_VALUE = std::numeric_limits<int>::max();
        OBB_max_CUBE_index_ = {MIN_VALUE, MIN_VALUE, MIN_VALUE};
        OBB_min_CUBE_index_ = {MAX_VALUE, MAX_VALUE, MAX_VALUE};
        for (auto list_ite = cubes_list_.begin(); list_ite != cubes_list_.end(); ) {
            OBB_max_CUBE_index_[0] = list_ite->first[0] > OBB_max_CUBE_index_[0] 
                                    ? list_ite->first[0] : OBB_max_CUBE_index_[0];
            OBB_max_CUBE_index_[1] = list_ite->first[1] > OBB_max_CUBE_index_[1] 
                                    ? list_ite->first[1] : OBB_max_CUBE_index_[1];
            OBB_max_CUBE_index_[2] = list_ite->first[2] > OBB_max_CUBE_index_[2] 
                                    ? list_ite->first[2] : OBB_max_CUBE_index_[2];
            OBB_min_CUBE_index_[0] = list_ite->first[0] < OBB_min_CUBE_index_[0] 
                                    ? list_ite->first[0] : OBB_min_CUBE_index_[0];
            OBB_min_CUBE_index_[1] = list_ite->first[1] < OBB_min_CUBE_index_[1] 
                                    ? list_ite->first[1] : OBB_min_CUBE_index_[1];
            OBB_min_CUBE_index_[0] = list_ite->first[2] < OBB_min_CUBE_index_[2] 
                                    ? list_ite->first[2] : OBB_min_CUBE_index_[2];
        }
        OBB_out_of_date_ = false;
    }

    // 获得有向包络盒的最大顶点的坐标。
    Eigen::Array3f GetOBBMaxCoordinate() const { 
        if (OBB_out_of_date_) {
            LOG(WARNING) << "OBB boundary maybe out of date!";
        }
        return (OBB_max_CUBE_index_ + Eigen::Array3i::Ones()).cast<float>()
                * GetCubeLength() * kCellSize;
    }

    // 获得有向包络盒的最小顶点的坐标。
    Eigen::Array3f GetOBBMinCoordinate() const { 
        if (OBB_out_of_date_) {
            LOG(WARNING) << "OBB boundary maybe out of date!";
        }
        return OBB_min_CUBE_index_.cast<float>()
                * GetCubeLength() * kCellSize;
    }

  private:

    // 通过 cell 的世界系三维索引，查找 cell 所在 CUBE 的世界系三维索引。
    Eigen::Array3i CellIndexToCubeIndex(const Eigen::Array3i& index) const 
    {
        // 考虑到索引可能为负数，std::floor() 是最佳的选择。
        // We have to consider the negative index!
        // The best way is to use floor() to always get the right CUBE index!
        return Eigen::Array3i{
            int(floor(index[0] * 1.f / CubeImpl::cube_length())),
            int(floor(index[1] * 1.f / CubeImpl::cube_length())),
            int(floor(index[2] * 1.f / CubeImpl::cube_length()))
        };
    }

    const float kCellSize;          // 一个 cell 的边长（米）
    const float kCellSizeInverse;   // 一个 cell 的边长的倒数
    const float kMaxRange;          // 最大维护距离当前中心点多远的cube

    Eigen::Vector3f curr_center_ = Eigen::Array3f::Zero();

    bool OBB_out_of_date_ = true;
    Eigen::Array3i OBB_max_CUBE_index_ = {0, 0, 0};   //当前地图的OBB边界
    Eigen::Array3i OBB_min_CUBE_index_ = {0, 0, 0};   //当前地图的OBB边界

    CellType exception_cell;        //mutable方式查询越界时返回一个无关紧要的值

    // 我们使用哈希表+双链表（unordered_map, list）来构成 IncrementalCube 的底层结构。
    // 理由在于：1-哈希表可以方便管理离散三维索引，<三维索引,CubeImpl*>构成哈希表的键值对；
    // 2-双链表支持在任意位置插入&删除元素，并且其它元素的增删不改变既有Iterator指向的元素。
    // 由于unordered_map的默认哈希函数支持C++内置类型，但不一定支持Eigen类型，所以我们
    // 使用自定义的空间哈希函数。
    using SpaceHashMap = std::unordered_map<
        Eigen::Array3i, 
        typename std::list<std::pair<Eigen::Array3i, std::unique_ptr<CubeImpl>>>::iterator, 
        SpaceHashFunc,
        IsEqualFunc>;

    SpaceHashMap cubes_map_;
    std::list<std::pair<Eigen::Array3i, std::unique_ptr<CubeImpl>>> cubes_list_;
};

}  // namespace common
}  // namespace infinityslam

#endif  // INFINITYSLAM_COMMON_INCREMENTAL_GRID_H_

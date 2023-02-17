/**
 * Copyright @ WANG_Guanhua ()
*/

#ifndef INFINITYSLAM_COMMON_MUTEX_AND_LOCK_H_
#define INFINITYSLAM_COMMON_MUTEX_AND_LOCK_H_

#include <mutex>
#include <thread>
// #include <boost/thread/thread.hpp>
#include <boost/thread.hpp>

namespace infinityslam {
namespace common {

using Mutex = std::mutex;
using UniqLock = std::unique_lock<std::mutex>;

using RWMutex = boost::shared_mutex;                        //读写互斥量
using ReadLock = boost::shared_lock<boost::shared_mutex>;   //共享读锁
using WriteLock = boost::unique_lock<boost::shared_mutex>;  //独占写锁

}
}


#endif // INFINITYSLAM_COMMON_MUTEX_AND_LOCK_H_
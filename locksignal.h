#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

typedef std::mutex Mutex;

class ThreadLock {
public:
    ThreadLock(Mutex& m);
    ThreadLock(const ThreadLock& other);
    
    void lock() { uniquelock.lock(); }
    void unlock() { uniquelock.unlock(); }
    
private:
    Mutex& m;
    std::unique_lock<Mutex> uniquelock;
};

class Signal {
public:
    Signal();
    Signal(const Signal& sig);
    ~Signal();
    
    void reset();
    
    void signal();
    
    void wait();
    void wait(int iTimeoutMS);
    
private:
    Mutex* pMutex;
    std::unique_lock<Mutex> lock;
    std::condition_variable cond;
};

inline ThreadLock::ThreadLock(Mutex& _m)
:   m(_m)
,   uniquelock(m, std::defer_lock)
{
}

inline ThreadLock::ThreadLock(const ThreadLock& other)
:   m(other.m)
,   uniquelock(m, std::defer_lock)
{
}

inline Signal::Signal()
: pMutex(new Mutex)
{
}

inline Signal::Signal(const Signal& sig)
: pMutex(nullptr)
{

}

inline Signal::~Signal()
{
    if (pMutex != nullptr) delete pMutex;
}
    
inline void Signal::reset()
{

}

inline void Signal::signal()
{

}

inline void Signal::wait()
{

}

inline void Signal::wait(int iTimeoutMS)
{

}
    
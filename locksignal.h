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
    
    void signal();
    
    void wait();
    void wait(int iTimeoutMS);
    
private:
    bool* pbReady;
    Mutex* pMutex;
    Mutex& mutex;
    std::condition_variable* pCond;
    std::unique_lock<Mutex> uniquelock;
};

inline ThreadLock::ThreadLock(Mutex& _m)
: m(_m)
, uniquelock(m, std::defer_lock)
{
}

inline ThreadLock::ThreadLock(const ThreadLock& other)
: m(other.m)
, uniquelock(m, std::defer_lock)
{
}

inline Signal::Signal()
: pbReady(new bool(false))
, pMutex(new Mutex)
, mutex(*pMutex)
, uniquelock(mutex, std::defer_lock)
, pCond(new std::condition_variable)
{
}

inline Signal::Signal(const Signal& sig)
: pbReady(sig.pbReady)
, mutex(sig.mutex)
, uniquelock(mutex, std::defer_lock)
, pCond(sig.pCond)
{

}

inline Signal::~Signal()
{
    if (pMutex != nullptr) {
        delete pbReady;
        delete pMutex;
        delete pCond;
    }
}

inline void Signal::signal()
{
    uniquelock.lock();
    *pbReady = true;
    uniquelock.unlock();
    pCond->notify_one();
}

inline void Signal::wait()
{
    uniquelock.lock();
    pCond->wait(uniquelock, [this]{return *pbReady;});
    *pbReady = false;
    uniquelock.unlock();
}

inline void Signal::wait(int iTimeoutMS)
{

}
    
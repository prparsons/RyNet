#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

typedef std::mutex ThreadMutex;

class ThreadLock {
public:
    ThreadLock(ThreadMutex& m);
    ThreadLock(const ThreadLock& other);
    
    void lock() { uniquelock.lock(); }
    void unlock() { uniquelock.unlock(); }
    
private:
    ThreadMutex& m;
    std::unique_lock<ThreadMutex> uniquelock;
};

class StopSignal {
public:
    StopSignal();
    StopSignal(const StopSignal& o);
    ~StopSignal();

    bool shouldStop();
    void stop();

    void lockForCopy();
    void unlockForCopy();

    void reset();

private:
    bool* pbStop;
    ThreadMutex* pm;
    int* piCount;
    std::unique_lock<ThreadMutex> ul;
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
    //Throws std::system_error when errors occur
    ThreadMutex* pThreadMutex;
    ThreadMutex& mutex;
    std::condition_variable* pCond;
    std::unique_lock<ThreadMutex> uniquelock;
};

inline ThreadLock::ThreadLock(ThreadMutex& _m)
: m(_m)
, uniquelock(m, std::defer_lock)
{
}

inline ThreadLock::ThreadLock(const ThreadLock& other)
: m(other.m)
, uniquelock(m, std::defer_lock)
{
}

inline StopSignal::StopSignal()
: pbStop(new bool(false))
, pm(new ThreadMutex)
, piCount(new int(1))
, ul(*pm, std::defer_lock)
{

}

inline StopSignal::StopSignal(const StopSignal& o)
: pbStop(o.pbStop)
, pm(o.pm)
, piCount(o.piCount)
, ul(*pm, std::defer_lock)
{
    ul.lock();
    *(piCount)++;
    ul.unlock();
}

inline StopSignal::~StopSignal()
{
    ul.lock();
    if (--(*piCount) == 0) {
        ul.unlock();
        delete pbStop;
        delete pm;
        delete piCount;
    } else {
        ul.unlock();
    }
}

inline bool StopSignal::shouldStop()
{
    std::lock_guard<ThreadMutex> lk(*pm);
    return (*pbStop);
}

inline void StopSignal::stop()
{
    std::lock_guard<ThreadMutex> lk(*pm);
    (*pbStop) = true;
}

inline void StopSignal::lockForCopy()
{
    ul.lock();
}

inline void StopSignal::unlockForCopy()
{
    ul.unlock();
}

inline void StopSignal::reset()
{
    std::lock_guard<ThreadMutex> lk(*pm);
    (*pbStop) = false;
}

inline Signal::Signal()
: pbReady(new bool(false))
, pThreadMutex(new ThreadMutex)
, mutex(*pThreadMutex)
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
    if (pThreadMutex != nullptr) {
        delete pbReady;
        delete pThreadMutex;
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
    uniquelock.lock();
    pCond->wait_for(uniquelock, std::chrono::milliseconds(iTimeoutMS), [this]{return *pbReady;});
    *pbReady = false;
    uniquelock.unlock();
}
    
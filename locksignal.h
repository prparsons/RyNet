#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>

typedef std::mutex ThreadMutex;

class StopSignal {
public:
    StopSignal();
    StopSignal(const StopSignal& o);
    ~StopSignal();

    bool shouldStop();
    void stop();

    void reset();

private:
    bool* pbStop;
    ThreadMutex* pThreadMutex;
    ThreadMutex& mutex;
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
};

inline StopSignal::StopSignal()
: pbStop(new bool(false))
, pThreadMutex(new ThreadMutex)
, mutex(*pThreadMutex)
{

}

inline StopSignal::StopSignal(const StopSignal& o)
: pThreadMutex(nullptr)
, mutex(o.mutex)
{
    std::lock_guard<ThreadMutex> lockGuard(mutex);
    pbStop = o.pbStop;
}

inline StopSignal::~StopSignal()
{
    if (pThreadMutex != nullptr) {
        delete pbStop;
        delete pThreadMutex;
    }
}

inline bool StopSignal::shouldStop()
{
    std::lock_guard<ThreadMutex> lk(mutex);
    return (*pbStop);
}

inline void StopSignal::stop()
{
    std::lock_guard<ThreadMutex> lk(mutex);
    (*pbStop) = true;
}

inline void StopSignal::reset()
{
    std::lock_guard<ThreadMutex> lk(mutex);
    (*pbStop) = false;
}

inline Signal::Signal()
: pbReady(new bool(false))
, pThreadMutex(new ThreadMutex)
, mutex(*pThreadMutex)
, pCond(new std::condition_variable)
{
}

inline Signal::Signal(const Signal& sig)
: pbReady(sig.pbReady)
, pThreadMutex(nullptr)
, mutex(sig.mutex)
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
    std::lock_guard<ThreadMutex> lockGuard(mutex);
    *pbReady = true;
    pCond->notify_one();
}

inline void Signal::wait()
{
    std::unique_lock<ThreadMutex> uniquelock(mutex);
    pCond->wait(uniquelock, [this]{return *pbReady;});
    *pbReady = false;
    uniquelock.unlock();
}

inline void Signal::wait(int iTimeoutMS)
{
    std::unique_lock<ThreadMutex> uniquelock(mutex);
    pCond->wait_for(uniquelock, std::chrono::milliseconds(iTimeoutMS), [this]{return *pbReady;});
    *pbReady = false;
    uniquelock.unlock();
}
    
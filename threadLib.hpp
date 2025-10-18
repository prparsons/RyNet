#pragma once

#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

typedef std::mutex ThreadMutex;

//DEPRECATED/////////////////////
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
////////////////////////////////

class Signal {
public:
    Signal(ThreadMutex& mt);
    Signal(const Signal& sig);
    ~Signal();
    
    void signal();
    void reset();
    void wait();
    void wait(int iTimeoutMS);
    
private:
    ThreadMutex& mutex;
    std::shared_ptr<bool> pbReady;
    std::shared_ptr<std::condition_variable> pCv;
};

template<class T>
class ThreadPipe
{
public:
    ThreadPipe(int iSizeArray);

    bool AddData(const T& data); // message passing (copying)
    bool AddData(T* pData); // shared memory (takes ownership, puts it in a shared_ptr)
    //bool AddData(T* pData, size_t uTimeout); // waits on signal if buffer isn't full (or signal is only reset when buffer fills?)
    bool RetrieveData(T& data, int iIdRecvr); // message passing
    bool RetrieveData(std::shared_ptr<T>& pData, int iIdRecvr); // shared memory

    int AddReceiver();
    void RemoveReceiver(int iRegistration);

    bool HasNewData(); // Whether there is new data in the pipe
    bool DataReady(int iIdRecvr); //New Data and this one hasn't read it yet (a read will succeed)

private:
    bool bThisRoundParity;
    size_t uNumReceivers; // If 0, each read will increment read index
    size_t uReadIndex;
    size_t uWriteIndex;
    size_t uWaitingCount; // How many pieces of data are waiting?
    std::vector<T> arData;
    std::vector<std::shared_ptr<T>> arSharedData;
    std::vector<std::pair<int, bool>> arReadThisRound;
    ThreadMutex mtData;
};

template<class T>
class ThreadPipeMsgPass
{
public:
    ThreadPipeMsgPass(std::shared_ptr<ThreadPipe<T>> _pipe): pipe(_pipe), iIdRecvr(-1) {}

    virtual bool AddData(const T& data) { return pipe->AddData(data); } // message passing (copying)
    virtual bool RetrieveData(T& data) { return pipe->RetrieveData(data, iIdRecvr); } // message passing

protected:
    void AddReceiver() { iIdRecvr = pipe->AddReceiver(); }
    void RemoveReceiver() { pipe->RemoveReceiver(iIdRecvr); }

    virtual bool AddData(T* pData) { return false; } // shared memory
    virtual bool RetrieveData(std::shared_ptr<T>& pData) { return false; } // shared memory 

    int iIdRecvr;
    std::shared_ptr<ThreadPipe<T>> pipe;
};

template<class T>
class ThreadPipeShared
{
public:
    ThreadPipeShared(std::shared_ptr<ThreadPipe<T>> _pipe): pipe(_pipe), iIdRecvr(-1) {}

    virtual bool AddData(T* pData) { return pipe->AddData(pData); } // shared memory
    virtual bool RetrieveData(std::shared_ptr<T>& pData) { return pipe->RetrieveData(pData, iIdRecvr); } // shared memory 

protected:
    void AddReceiver() { iIdRecvr = pipe->AddReceiver(); }
    void RemoveReceiver() { pipe->RemoveReceiver(iIdRecvr); }

    virtual bool AddData(const T& data) { return false; } // message passing (copying)
    virtual bool RetrieveData(T& data) { return false; } // message passing

    int iIdRecvr;
    std::shared_ptr<ThreadPipe<T>> pipe;
};

template<class T, class P> class ThreadPipeReceiver;

template<class T, class P>
class ThreadPipeSender: public P {
public:
    ThreadPipeSender(int iSize): P(std::make_shared<ThreadPipe<T>>(iSize)) {}

    ThreadPipeReceiver<T, P>* CreateReceiver(bool bRequiredReceiver) { return new ThreadPipeReceiver<T, P>(P::pipe, bRequiredReceiver); }
    ThreadPipeSender<T, P>* CreateSender() { return new ThreadPipeSender<T, P>(P::pipe); }

protected:
    // Both sends are left public, only P's will work. Override both recvs (we don't know which one is right)
    // and have them both fail. So this will only send the 1 way.
    bool RetrieveData(T& data) override { return false; } // message passing
    bool RetrieveData(std::shared_ptr<T>& pData) override { return false; } // shared memory

private:
    // This is inherited from either shared or message passing pipe. So this constructor takes the base
    // pipe as a parameter, sends that to either shared or message passing constructor.
    ThreadPipeSender(std::shared_ptr<ThreadPipe<T>>& pipe): P(pipe) {}

    friend class ThreadPipeReceiver<T, P>;
};

template<class T, class P>
class ThreadPipeReceiver: public P {
public:
    ThreadPipeReceiver(int iSize, bool bRequiredReceiver);
    ~ThreadPipeReceiver();

    ThreadPipeReceiver<T, P>* CreateReceiver(bool bRequiredReceiver);
    ThreadPipeSender<T, P>* CreateSender();

    bool HasNewData();
    bool DataReady();

protected:
    bool AddData(const T& data) override; // message passing (copying)
    bool AddData(T* pData) override; // shared memory (takes ownership, puts it in a shared_ptr)

private:
    ThreadPipeReceiver(std::shared_ptr<ThreadPipe<T>>& pipe, bool bRequiredReceiver);

    bool bRequiredReceiver;

    friend ThreadPipeReceiver<T, P>* ThreadPipeSender<T, P>::CreateReceiver(bool);
};

/////////////////////////////////////////////////////////////////////
template<class T>
ThreadPipe<T>::ThreadPipe(int iSizeArray)
: arData(iSizeArray)
, arSharedData(iSizeArray)
, uNumReceivers(0)
, uReadIndex(0)
, uWriteIndex(0)
, uWaitingCount(0)
{
}

// 
template<class T>
bool ThreadPipe<T>::AddData(const T& data)
{
    std::lock_guard<ThreadMutex> lk(mtData);

    // Check if buffer is full
    if (uWaitingCount == arData.size()) return false;

    arData[uWriteIndex++] = data;
    if (uWriteIndex == arData.size())
        uWriteIndex = 0;

    uWaitingCount++;
    return true;
} // message passing (copying)

template<class T>
bool ThreadPipe<T>::AddData(T* pData)
{
    if (pData == nullptr) return false;
    
    std::lock_guard<ThreadMutex> lk(mtData);

    // Check if buffer is full
    if (uWaitingCount == arSharedData.size()) return false;

    arSharedData[uWriteIndex].reset(pData);
    if (uWriteIndex == arSharedData.size())
        uWriteIndex = 0;

    uWaitingCount++;
    return true;
}

template<class T>
bool ThreadPipe<T>::RetrieveData(T& data, int iIdRecvr)
{
    //todo Both of these need to fail if read index has outpaced write index some how
    std::lock_guard<ThreadMutex> lk(mtData);

    if (iIdRecvr == -1) {
        // If this isn't a required receiver, it just gets latest data
        if (uWriteIndex == 0) {
            data = arData[arData.size() - 1];
        } else {
            data = arData[uWriteIndex - 1];
        }
        return true;
    }

    if (uWaitingCount == 0) return false; // Nothing new yet
    //todo ^This is where reading could wait on a signal

    // Get the data
    data = arData[uReadIndex];
    // See if this was the last one to be read
    bool bAllOthersHaveRead = true;
    for (size_t u = 0; u < uNumReceivers; u++) {
        // Set this one as "it's been read"
        if (arReadThisRound[u].first == iIdRecvr) {
            if (arReadThisRound[u].second == bThisRoundParity) {
                // If this has read it before, then at least one other is still waiting to read, 
                // otherwise we'd have moved on
                return false;
            }
            arReadThisRound[u].second = bThisRoundParity;
        }
        // Check all the others
        else if (arReadThisRound[u].second != bThisRoundParity) {
            bAllOthersHaveRead = false;
        }
    }
    if (!bAllOthersHaveRead) {
        return true;
    }
    // If all have been read, this can't be a re-read, because otherwise the last call
    // would have moved us to the next value.

    // All have been read now
    bThisRoundParity = !bThisRoundParity;
    uWaitingCount--;
    uReadIndex++;
    //todo a signal here would let any waiting senders know that there was a spot (instead of a naive wait/loop implementation)
    if (uReadIndex == arData.size())
        uReadIndex = 0;

    return true;
}

template<class T>
bool ThreadPipe<T>::RetrieveData(std::shared_ptr<T>& pData, int iIdRecvr)
{
    std::lock_guard<ThreadMutex> lk(mtData);

    if (iIdRecvr == -1) {
        // If this isn't a required receiver, it just gets latest data
        pData = arSharedData[uWriteIndex];
        return true;
    }
    
    if (uWaitingCount == 0) return false;
    //todo ^This is where reading could wait on a signal

    // Get the data
    pData = arSharedData[uReadIndex];
    // See if this was the last one to be read
    bool bAllOthersHaveRead = true;
    for (size_t u = 0; u < uNumReceivers; u++) {
        // Set this one as "it's been read"
        if (arReadThisRound[u].first == iIdRecvr) {
            if (arReadThisRound[u].second == bThisRoundParity) {
                // If this has read it before, then at least one other is still waiting to read, 
                // otherwise we'd have moved on
                return false;
            }
            arReadThisRound[u].second = bThisRoundParity;
        }
        // Check all the others
        else if (arReadThisRound[u].second != bThisRoundParity) {
            bAllOthersHaveRead = false;
        }
    }
    if (!bAllOthersHaveRead) {
        return true;
    }
    // If all have been read, this can't be a re-read, because otherwise the last call
    // would have moved us to the next value.

    // All have been read now
    bThisRoundParity = !bThisRoundParity;
    uWaitingCount--;
    uReadIndex++;
    if (uReadIndex == arSharedData.size())
        uReadIndex = 0;

    return true;
} // shared memory

template<class T>
int ThreadPipe<T>::AddReceiver()
{
    std::lock_guard<ThreadMutex> lock(mtData);

    std::pair<int, bool> newRecvr(int(uNumReceivers++), !bThisRoundParity);   
    arReadThisRound.push_back(newRecvr);
    return newRecvr.first;
}

template<class T>
void ThreadPipe<T>::RemoveReceiver(int iIdRecvr)
{
    std::lock_guard<ThreadMutex> lock(mtData);

    for (std::vector<std::pair<int, bool>>::iterator iter = arReadThisRound.begin(); iter != arReadThisRound.end(); iter++) {
        if (iter->first == iIdRecvr) {
            arReadThisRound.erase(iter);
            uNumReceivers--;
            break;
        }
    }
}

template<class T>
bool ThreadPipe<T>::HasNewData()
{
    std::lock_guard<ThreadMutex> lock(mtData);
    return uWaitingCount > 0;
}

template<class T>
bool ThreadPipe<T>::DataReady(int iIdRecvr)
{
    std::lock_guard<ThreadMutex> lock(mtData);
    if (uWaitingCount == 0) return false;

    for (size_t u = 0; u < uNumReceivers; u++) {
        if (arReadThisRound[u].first == iIdRecvr) {
            return arReadThisRound[u].second != bThisRoundParity;
        }
    }
    return false;
}

template<class T, class P>
ThreadPipeReceiver<T, P>::ThreadPipeReceiver(int iSize, bool _bRequiredReceiver)
: P(std::make_shared<ThreadPipe<T>>(iSize))
, bRequiredReceiver(_bRequiredReceiver)
{
    if (bRequiredReceiver)
        P::AddReceiver();
}

template<class T, class P>
ThreadPipeReceiver<T, P>::~ThreadPipeReceiver()
{
    if (bRequiredReceiver)
        P::RemoveReceiver();
}

template<class T, class P>
ThreadPipeReceiver<T, P>* ThreadPipeReceiver<T, P>::CreateReceiver(bool bRequiredReceiver)
{
    return new ThreadPipeReceiver<T, P>(P::pipe, bRequiredReceiver);
}

template<class T, class P>
ThreadPipeSender<T, P>* ThreadPipeReceiver<T, P>::CreateSender()
{
    return new ThreadPipeSender<T, P>(P::pipe);
}

template<class T, class P>
bool ThreadPipeReceiver<T, P>::HasNewData()
{
    if (!bRequiredReceiver) return true; //todo wrong if pipe has never had anything
    return P::pipe.HasNewData();
}

template<class T, class P>
bool ThreadPipeReceiver<T, P>::DataReady()
{
    if (!bRequiredReceiver) return true;
    return P::pipe.DataReady(P::iIdRecvr);
}

template<class T, class P>
bool ThreadPipeReceiver<T, P>::AddData(const T& data)
{
    return false;
}

template<class T, class P>
bool ThreadPipeReceiver<T, P>::AddData(T* pData)
{
    return false;
}

template<class T, class P>
ThreadPipeReceiver<T, P>::ThreadPipeReceiver(std::shared_ptr<ThreadPipe<T>>& pipe, bool _bRequiredReceiver)
: P(pipe)
, bRequiredReceiver(_bRequiredReceiver)
{
    if (bRequiredReceiver)
        P::AddReceiver();
}

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

inline Signal::Signal(ThreadMutex& mt)
: mutex(mt)
, pbReady(new bool(false))
, pCv(new std::condition_variable)
{
}

inline Signal::Signal(const Signal& sig)
: mutex(sig.mutex)
, pbReady(sig.pbReady)
, pCv(sig.pCv)
{

}

inline Signal::~Signal()
{
}

inline void Signal::signal()
{
    std::unique_lock<ThreadMutex> lock(mutex);
    *pbReady = true;
    lock.unlock();

    pCv->notify_all();
}

inline void Signal::reset()
{
    std::lock_guard<ThreadMutex> lock(mutex);
    *pbReady = false;
}

inline void Signal::wait()
{
    std::unique_lock<ThreadMutex> uniquelock(mutex);

    if (*pbReady) return;
    pCv->wait(uniquelock, [this]{return *pbReady;});

    uniquelock.unlock();
}

inline void Signal::wait(int iTimeoutMS)
{
    std::unique_lock<ThreadMutex> uniquelock(mutex);

    if (*pbReady) return;
    pCv->wait_for(uniquelock, std::chrono::milliseconds(iTimeoutMS), [this]{return *pbReady;});

    uniquelock.unlock();
}
    
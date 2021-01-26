#pragma once

#include <thread>

using namespace std;

class ServerLoop {
public:
    ServerLoop();

    void operator()(void);

private:
};

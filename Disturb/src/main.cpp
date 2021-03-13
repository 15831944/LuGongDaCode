#include <csignal>

#include "device/Disturb.h"

using namespace std;

bool loop = true;

void handler_cancel(int s) {
    LOG(INFO) << "killed by signal :" << s;
    loop = false;
}



int main()
{
    std::signal(SIGINT, handler_cancel);

    Disturb Disturb;
    Disturb.Start();

    while(loop){
        msleep(1);
    }




    return 0;
}











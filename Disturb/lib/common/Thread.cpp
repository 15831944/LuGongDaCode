#include <common/Thread.h>

void Thread::start(void *(*task)(void*), void *arg)
{
	assert(mThread==((pthread_t)0));
	assert(!pthread_attr_init(&mAttrib));
	assert(!pthread_attr_setstacksize(&mAttrib, mStackSize));
	assert(!pthread_create(&mThread, &mAttrib, task, arg));
}


Mutex::Mutex()
{
	assert(!pthread_mutexattr_init(&mAttribs));
	assert(!pthread_mutexattr_settype(&mAttribs,PTHREAD_MUTEX_RECURSIVE));
	assert(!pthread_mutex_init(&mMutex,&mAttribs));
}


Mutex::~Mutex()
{
	pthread_mutex_destroy(&mMutex);
	assert(!pthread_mutexattr_destroy(&mAttribs));
}

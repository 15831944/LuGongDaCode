#ifndef THREAD_HPP
#define THREAD_HPP


#include <pthread.h>
#include <assert.h>
class Thread {

	private:

	pthread_t mThread;
	pthread_attr_t mAttrib;
	// FIXME -- Can this be reduced now?
	size_t mStackSize;

	public:

	/** Create a thread in a non-running state. */
	Thread(size_t wStackSize = (65536*4)):mThread((pthread_t)0) { mStackSize=wStackSize;}

	/**
		Destroy the Thread.
		It should be stopped and joined.
	*/
	~Thread() { assert(!pthread_attr_destroy(&mAttrib)); }


	/** Start the thread on a task. */
	void start(void *(*task)(void*), void *arg);

	/** Join a thread that will stop on its own. */
	void join() { assert(!pthread_join(mThread,NULL)); }

};


class Mutex {

	private:

	pthread_mutex_t mMutex;
	pthread_mutexattr_t mAttribs;

	public:

	Mutex();

	~Mutex();

	void lock() { pthread_mutex_lock(&mMutex); }

	void unlock() { pthread_mutex_unlock(&mMutex); }

	friend class Signal;

};

#endif

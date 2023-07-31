/*

   BLIS
   An object-based framework for developing high-performance BLAS-like
   libraries.

   Copyright (C) 2018, Southern Methodist University
   Copyright (C) 2018, The University of Texas at Austin
   Copyright (C) 2018 - 2019, Advanced Micro Devices, Inc.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:
    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    - Neither the name(s) of the copyright holder(s) nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "blis.h"

#if defined(BLIS_ENABLE_HPX)

#include "bli_pthread_hpx.h"

#include <hpx/condition_variable.hpp>
#include <hpx/mutex.hpp>
#include <hpx/thread.hpp>
#include <hpx/synchronization/once.hpp>
#include <hpx/barrier.hpp>

#if defined(__cplusplus)
extern "C" {
#endif

struct bli_pthread_ {
   std::shared_ptr<hpx::thread> t;
   void* retval;
};

struct bli_pthread_mutex_ {
    hpx::spinlock m;
};

struct bli_pthread_cond_ {
    hpx::condition_variable cv;
};

struct bli_pthread_once_ {
    hpx::once_flag f;
};

struct bli_pthread_switch_
{
    int                 status;
    bli_pthread_mutex_t mutex;
};

struct bli_pthread_barrier_ {
    std::shared_ptr<hpx::barrier<>> b;
};

// This branch defines a pthreads-like API, bli_pthreads_*(), and implements it
// in terms of the corresponding pthreads_*() types, macros, and function calls. 
// This branch is compiled for Linux and other non-Windows environments where
// we assume that *some* implementation of pthreads is provided (although it
// may lack barriers--see below).

// -- pthread_create(), pthread_join() --

int bli_pthread_create
     (
       bli_pthread_t*            thread,
       const bli_pthread_attr_t* attr,
       void*                   (*start_routine)(void*),
       void*                     arg
     )
{
	//return pthread_create( thread, attr, start_routine, arg );
        thread->t = std::make_shared<hpx::thread>(std::bind(start_routine, arg));
        return 1;
}

int bli_pthread_join
     (
       bli_pthread_t thread,
       void**        retval
     )
{
        thread.t->join();
	if ( retval ) *retval = thread.retval;
	return 0;
}

// -- pthread_mutex_*() --

int bli_pthread_mutex_init
     (
       bli_pthread_mutex_t*           mutex,
       const bli_pthread_mutexattr_t* attr
     )
{
	if ( attr ) return EINVAL;
	return 0;
}

int bli_pthread_mutex_destroy
     (
       bli_pthread_mutex_t* mutex
     )
{
	return 0;
}

int bli_pthread_mutex_lock
     (
       bli_pthread_mutex_t* mutex
     )
{
        mutex->m.lock();

	return 0;
}

int bli_pthread_mutex_trylock
     (
       bli_pthread_mutex_t* mutex
     )
{
        mutex->m.try_lock();

	return 0;
}

int bli_pthread_mutex_unlock
     (
       bli_pthread_mutex_t* mutex
     )
{
        mutex->m.unlock();

	return 0;
}

// -- pthread_cond_*() --

int bli_pthread_cond_init
     (
       bli_pthread_cond_t*           cond,
       const bli_pthread_condattr_t* attr
     )
{
	return 0;
}

int bli_pthread_cond_destroy
     (
       bli_pthread_cond_t* cond
     )
{
	return 0;
}

int bli_pthread_cond_wait
     (
       bli_pthread_cond_t*  cond,
       bli_pthread_mutex_t* mutex
     )
{
        std::unique_lock<hpx::spinlock> lk(mutex->m);
        cond->cv.wait(lk);
        return 0;
}

int bli_pthread_cond_broadcast
     (
       bli_pthread_cond_t* cond
     )
{
        cond->cv.notify_all();
	return 0;
}

// -- pthread_once() --

void bli_pthread_once
     (
       bli_pthread_once_t* once,
       void              (*init)(void)
     )
{
        hpx::call_once(once->f, init);
}

// Linux environments implement the pthread_barrier* sub-API. So, if we're
// on Linux, we can simply call those functions, just as we did before for
// the other functions.

int bli_pthread_barrier_init
     (
       bli_pthread_barrier_t*           barrier,
       const bli_pthread_barrierattr_t* attr,
       unsigned int                     count
     )
{

        barrier->b = std::make_shared<hpx::barrier<>>(count);
	return 0;
}

int bli_pthread_barrier_destroy
     (
       bli_pthread_barrier_t* barrier
     )
{
	return 0;
}

int bli_pthread_barrier_wait
     (
       bli_pthread_barrier_t* barrier
     )
{
        barrier->b->arrive_and_wait();
	return 0;
}

int bli_pthread_switch_on
     (
       bli_pthread_switch_t* sw,
       int                 (*init)(void)
     )
{
	// NOTE: This function assumes that init() will return 0 on success;
	// otherwise, it will return some other integer. If the function
	// partially succeeds (in such a way that it must be called again in
	// order to complete), it should treat that outcome as failure and
	// return a non-zero value.

	// Initialize the return value with the error code for success.
	int r_val = 0;

	// Proceed only if the switch is currently off; otherwise, we return with
	// an error code of 0.
	if ( sw->status == 0 )
	{
		// Wait for and acquire the switch's lock.
		bli_pthread_mutex_lock( &sw->mutex );

		// Check the status of the switch once more now that we've acquired the
		// lock. Proceed with calling the init() function only if the switch
		// is still off; otherwise, release the lock with an error code of 0.
		if ( sw->status == 0 )
		{
			// Call the init() function and catch its return value in r_val.
			r_val = init();

			// If the init() function succeeded, turn the switch on;
			// otherwise, leave the switch off.
			if ( r_val == 0 )
				sw->status = 1;
		}

		// Release the switch's lock.
		bli_pthread_mutex_unlock( &sw->mutex );
	}

	return r_val;
}

int bli_pthread_switch_off
     (
       bli_pthread_switch_t* sw,
       int                 (*deinit)(void)
     )
{
	// NOTE: This function assumes that deinit() will return 0 on success;
	// otherwise, it will return some other integer. If the function
	// partially succeeds (in such a way that it must be called again in
	// order to complete), it should treat that outcome as failure and
	// return a non-zero value.

	// Initialize the return value with the error code for success.
	int r_val = 0;

	// Proceed only if the switch is currently on; otherwise, we return with
	// an error code of 0.
	if ( sw->status == 1 )
	{
		// Wait for and acquire the switch's lock.
		bli_pthread_mutex_lock( &sw->mutex );

		// Check the status of the switch once more now that we've acquired the
		// lock. Proceed with calling the deinit() function only if the switch
		// is still on; otherwise, release the lock with an error code of 0.
		if ( sw->status == 1 )
		{
			// Call the deinit() function and catch its return value in r_val.
			r_val = deinit();

			// If the deinit() function succeeded, turn the switch off;
			// otherwise, leave the switch on.
			if ( r_val == 0 )
				sw->status = 0;
		}

		// Release the switch's lock.
		bli_pthread_mutex_unlock( &sw->mutex );
	}

	return r_val;
}

static bli_pthread_mutex_t gks_mutex;

bli_pthread_mutex_t * get_gks_mutex_ptr() { return &gks_mutex; }

#if defined(__cplusplus)
}
#endif

#endif

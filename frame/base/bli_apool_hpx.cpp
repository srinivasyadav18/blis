/*

   BLIS
   An object-based framework for developing high-performance BLAS-like
   libraries.

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

#if defined(__cplusplus) && defined(BLIS_ENABLE_HPX)

#include<memory>
#include <hpx/mutex.hpp>

#include "bli_apool_hpx.h"

extern "C" {

struct bli_pthread_mutex_ {
    hpx::spinlock m;
};

struct apool_
{
	std::shared_ptr<bli_pthread_mutex_t> mutex;
	pool_t              pool;

	siz_t               def_array_len;
};

static apool_t sba;

pool_t* bli_apool_pool( apool_t* apool )
{
	return &(apool->pool);
}

bli_pthread_mutex_t* bli_apool_mutex( apool_t* apool )
{
	return apool->mutex.get();
}

siz_t bli_apool_def_array_len( const apool_t* pool )
{
	return pool->def_array_len;
}

bool bli_apool_is_exhausted( const apool_t* apool )
{
	return bli_pool_is_exhausted( &apool->pool );
}

void bli_apool_set_def_array_len( siz_t def_array_len, apool_t* pool ) \
{
	pool->def_array_len = def_array_len;
}

void bli_apool_init
     (
       apool_t* apool
     )
{
	err_t r_val;

	// NOTE: The apool_t is only used in one place; it is the type used to
	// define the sba. We've switched to static initialization of the mutex
	// field to remove one more thing that could possibly go wrong during
	// library initialization.

	// Query the mutex from the apool_t.
	//bli_pthread_mutex_t* mutex = bli_apool_mutex( apool );
        apool->mutex = std::make_shared<bli_pthread_mutex_t>();

	// Initialize the mutex.
	//*mutex = BLIS_PTHREAD_MUTEX_INITIALIZER;
	//bli_pthread_mutex_init( mutex, NULL );

	// We choose to start with:
	// - an empty pool
	// - an initial block_ptrs_len of 8
	// - a single element in each initial array_t (though this is moot with
	//   num_blocks = 0).
	const siz_t num_blocks     = 0;
	      siz_t block_ptrs_len = 8;
	const siz_t num_elem       = 1;

	// NOTE: Unlike in the bli_pool API, apool_t allocates block_ptrs as an
	// array of array_t* instead of an array of pblk_t. Why? We don't need to
	// track the size of each block, thus we don't need the block_size field
	// of pblk_t. That leaves only the void* field, and since we know apool_t
	// will always contain "blocks" that are really array_t structs, we can
	// make block_ptrs an array of array_t*.

	// We formally set the block_size and align_size fields of the underlying
	// pool, even though they won't be queried. (They are used from hard-coded
	// values in bli_apool_alloc_block().)
	const siz_t block_size = sizeof( array_t );
	const siz_t align_size = 64;

	// Query the underlying pool_t from the apool_t.
	pool_t* pool = bli_apool_pool( apool );

	// Set the default array_t length of the apool_t.
	bli_apool_set_def_array_len( num_elem, apool );

	// -------------------------------------------------------------------------

	// Make sure that block_ptrs_len is at least num_blocks.
	block_ptrs_len = bli_max( block_ptrs_len, num_blocks );

	#ifdef BLIS_ENABLE_MEM_TRACING
	printf( "bli_apool_init(): allocating block_ptrs (length %d): ",
	        ( int )block_ptrs_len );
	#endif

	// Allocate the block_ptrs array.
	array_t** block_ptrs
	=
	static_cast<array_t**>(bli_malloc_intl( block_ptrs_len * sizeof( array_t* ), &r_val ));

	#ifdef BLIS_ENABLE_MEM_TRACING
	printf( "bli_apool_init(): allocating %d array_t.\n", ( int )num_blocks );
	fflush( stdout );
	#endif

	// Allocate and initialize each entry in the block_ptrs array.
	for ( dim_t i = 0; i < num_blocks; ++i )
	{
		// Pass in num_elem so the function knows how many elements to
		// initially have in each array_t.
		bli_apool_alloc_block
		(
		  num_elem,
		  &(block_ptrs[i])
		);
	}

	// NOTE: The semantics of top_index approximate a stack, where a "full"
	// stack (no blocks checked out) is one where top_index == 0 and an empty
	// stack (all blocks checked out) one where top_index == num_blocks.
	// (Here, num_blocks tracks the number of blocks currently allocated as
	// part of the pool.) This "orientation" of the stack was chosen
	// intentionally, in contrast to one where top_index == -1 means the
	// stack is empty and top_index = num_blocks - 1 means the stack is
	// full. The chosen scheme allows one to conceptualize the stack as a
	// number line in which blocks are checked out from lowest to highest,
	// and additional blocks are added at the higher end.

	// Initialize the pool_t structure.
	// NOTE: We don't use the malloc_fp and free_fp fields at the apool_t
	// level. Nevertheless, we set them to NULL.
	bli_pool_set_block_ptrs( block_ptrs, pool );
	bli_pool_set_block_ptrs_len( block_ptrs_len, pool );
	bli_pool_set_top_index( 0, pool );
	bli_pool_set_num_blocks( num_blocks, pool );
	bli_pool_set_block_size( block_size, pool );
	bli_pool_set_align_size( align_size, pool );
	bli_pool_set_malloc_fp( NULL, pool );
	bli_pool_set_free_fp( NULL, pool );
}

}

#endif

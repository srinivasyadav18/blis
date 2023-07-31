/*

   BLIS
   An object-based framework for developing high-performance BLAS-like
   libraries.

   Copyright (C) 2014, The University of Texas at Austin
   Copyright (C) 2016, Hewlett Packard Enterprise Development LP
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

extern "C" {

struct bli_pthread_mutex_ {
    hpx::spinlock m;
};

struct pool_
{
	void*     block_ptrs;
	dim_t     block_ptrs_len;

	dim_t     top_index;
	dim_t     num_blocks;

	siz_t     block_size;
	siz_t     align_size;
	siz_t     offset_size;

	malloc_ft malloc_fp;
	free_ft   free_fp;

};

struct pba_s
{
	pool_t              pools[3];
	std::shared_ptr<bli_pthread_mutex_t> mutex;

	// These fields are used for general-purpose allocation.
	siz_t               align_size;
	malloc_ft           malloc_fp;
	free_ft             free_fp;

};

pba_t global_pba{.mutex=std::make_shared<bli_pthread_mutex_t>()};

pool_t* bli_pba_pool( dim_t pool_index, pba_t* pba )
{
	return &(pba->pools[ pool_index ]);
}

siz_t bli_pba_align_size( const pba_t* pba )
{
	return pba->align_size;
}

malloc_ft bli_pba_malloc_fp( const pba_t* pba )
{
	return pba->malloc_fp;
}

free_ft bli_pba_free_fp( const pba_t* pba )
{
	return pba->free_fp;
}

// pba modification

void bli_pba_set_align_size( siz_t align_size, pba_t* pba )
{
	pba->align_size = align_size;
}

void bli_pba_set_malloc_fp( malloc_ft malloc_fp, pba_t* pba )
{
	pba->malloc_fp = malloc_fp;
}

void bli_pba_set_free_fp( free_ft free_fp, pba_t* pba )
{
	pba->free_fp = free_fp;
}

// pba action

void bli_pba_lock( pba_t* pba )
{
	bli_pthread_mutex_lock( pba->mutex.get() );
}

void bli_pba_unlock( pba_t* pba )
{
	bli_pthread_mutex_unlock( pba->mutex.get() );
}

}

#endif

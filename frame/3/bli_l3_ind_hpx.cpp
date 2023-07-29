/*

   BLIS
   An object-based framework for developing high-performance BLAS-like
   libraries.

   Copyright (C) 2014, The University of Texas at Austin
   Copyright (C) 2018 - 2020, Advanced Micro Devices, Inc.

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

#include <hpx/mutex.hpp>
#include <hpx/thread.hpp>

//
// NOTE: "2" is used instead of BLIS_NUM_FP_TYPES/2.
//
// BLIS provides APIs to modify this state during runtime. So, it's possible for one
// application thread to modify the state before another starts the corresponding
// BLIS operation. This is solved by making the induced method status array local to
// threads.

static BLIS_THREAD_LOCAL
bool bli_l3_ind_oper_st[BLIS_NUM_IND_METHODS][BLIS_NUM_LEVEL3_OPS][2] =
{
        /*   gemm           gemmt          hemm           herk           her2k          symm
             syrk           syr2k          trmm3          trmm           trsm  */
        /*    c     z    */
/* 1m   */ { {FALSE,FALSE}, {FALSE,FALSE}, {FALSE,FALSE}, {FALSE,FALSE}, {FALSE,FALSE}, {FALSE,FALSE},
             {FALSE,FALSE}, {FALSE,FALSE}, {FALSE,FALSE}, {FALSE,FALSE}, {FALSE,FALSE}  },
/* nat  */ { {TRUE,TRUE},   {TRUE,TRUE},   {TRUE,TRUE},   {TRUE,TRUE},   {TRUE,TRUE},   {TRUE,TRUE},
             {TRUE,TRUE},   {TRUE,TRUE},   {TRUE,TRUE},   {TRUE,TRUE},   {TRUE,TRUE}    },
};

struct bli_pthread_mutex_ {
    hpx::spinlock m;
};

static bli_pthread_mutex_t oper_st_mutex;

#if defined(__cplusplus) 
extern "C" {
#endif

void bli_l3_ind_oper_set_enable( opid_t oper, ind_t method, num_t dt, bool status )
{
	num_t idt;

	if ( !bli_is_complex( dt ) ) return;
	if ( !bli_opid_is_level3( oper ) ) return;

	// BLIS currently implements herk/her2k/syrk/syr2k in terms of the user-
	// level gemmt (expert) API, and so those operations choose to execute
	// 1m (or not) based on the induced method enablement status of gemmt.
	// In other words, changing the enablement status of those operations
	// would have no effect. Therefore, we redirect queries/accesses to those
	// operations' induced method enablement statuses to that of gemmt.
	if ( method != BLIS_NAT && ( oper == BLIS_HERK  ||
	                             oper == BLIS_HER2K ||
	                             oper == BLIS_SYRK  ||
	                             oper == BLIS_SYR2K ) )
		oper = BLIS_GEMMT;

	// Disallow changing status of native execution.
	if ( method == BLIS_NAT ) return;

	idt = bli_ind_map_cdt_to_index( dt );

	// Acquire the mutex protecting bli_l3_ind_oper_st.
        std::lock_guard sl(oper_st_mutex.m);

	// BEGIN CRITICAL SECTION
	{
		bli_l3_ind_oper_st[ method ][ oper ][ idt ] = status;
	}
	// END CRITICAL SECTION
}

#if defined(__cplusplus) 
}
#endif

#endif

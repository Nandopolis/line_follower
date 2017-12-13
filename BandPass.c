/******************************* SOURCE LICENSE *********************************
Copyright (c) 2015 MicroModeler.

A non-exclusive, nontransferable, perpetual, royalty-free license is granted to the Licensee to 
use the following Information for academic, non-profit, or government-sponsored research purposes.
Use of the following Information under this License is restricted to NON-COMMERCIAL PURPOSES ONLY.
Commercial use of the following Information requires a separately executed written license agreement.

This Information is distributed WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

******************************* END OF LICENSE *********************************/

// A commercial license for MicroModeler DSP can be obtained at http://www.micromodeler.com/launch.jsp

#include "BandPass.h"

#include <stdlib.h> // For malloc/free
#include <string.h> // For memset

float32_t BandPass_coefficients[5] = 
{
// Scaled for floating point

    0.05920187205475464, 0, -0.05920187205475464, 1.793063825321928, -0.8816185923631894// b0, b1, b2, a1, a2

};


BandPassType *BandPass_create( void )
{
	BandPassType *result = (BandPassType *)malloc( sizeof( BandPassType ) );	// Allocate memory for the object
	BandPass_init( result );											// Initialize it
	return result;																// Return the result
}

void BandPass_destroy( BandPassType *pObject )
{
	free( pObject );
}

 void BandPass_init( BandPassType * pThis )
{
	arm_biquad_cascade_df1_init_f32(	&pThis->instance, BandPass_numStages, BandPass_coefficients, pThis->state );
	BandPass_reset( pThis );

}

 void BandPass_reset( BandPassType * pThis )
{
	memset( &pThis->state, 0, sizeof( pThis->state ) ); // Reset state to 0
	pThis->output = 0;									// Reset output

}

 int BandPass_filterBlock( BandPassType * pThis, float * pInput, float * pOutput, unsigned int count )
{
	arm_biquad_cascade_df1_f32( &pThis->instance, pInput, pOutput, count );
	return count;

}



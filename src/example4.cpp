/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file examples/example4.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2009-2017
 *
 *	Very simple example for testing qpOASES (using the possibility to specify 
 *	user-defined constraint product function).
 */



#include <stdlib.h>

#include <qpOASES.hpp>

BEGIN_NAMESPACE_QPOASES


/** 
 *	\brief Example illustrating the use of the \a ConstraintProduct class.
 *
 *	Example illustrating the use of the \a ConstraintProduct class.
 *
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2007-2017
 */
class MyConstraintProduct : public ConstraintProduct
{
	public:
		/** Default constructor. */
		MyConstraintProduct() {};

		/** Constructor. */
		MyConstraintProduct(	int_t _nV,
								int_t _nC,
								real_t* _A
								)
		{
			nV = _nV;
			nC = _nC;
			A  = _A;
		};

		/** Copy constructor (flat copy). */
		MyConstraintProduct(const MyConstraintProduct& rhs) : ConstraintProduct(rhs) // 기본 클래스의 복사 생성자를 명시적으로 호출
		{
		    nV = rhs.nV;
		    nC = rhs.nC;
		    A  = rhs.A;
		};

		/** Destructor. */
		virtual ~MyConstraintProduct() {};
		
		/** Assignment operator (flat copy). */
		MyConstraintProduct& operator=(	const MyConstraintProduct& rhs
										)
		{
			if ( this != &rhs )
			{
				nV = rhs.nV;
				nC = rhs.nC;
				A  = rhs.A;
			}
			return *this;
		};

		virtual int_t operator() (	int_t constrIndex,
									const real_t* const x,
									real_t* const constrValue
									) const
		{
			int_t i;

			constrValue[0] = 1.0 * x[(constrIndex/10)+2];

			for( i=0; i<2; ++i )
				constrValue[0] += A[constrIndex*nV + i] * x[i];

			return 0;
		};

	protected:
		int_t nV;		/**< Number of variables. */
		int_t nC;		/**< Number of constraints. */
		real_t* A;		/**< Pointer to full constraint matrix (typically not needed!). */
};


END_NAMESPACE_QPOASES

/**	Example for qpOASES main function using the possibility to specify 
 *	user-defined constraint product function. */
int main( )
{
	USING_NAMESPACE_QPOASES

	int_t i,j;

	/* Setup data of first QP... */
	real_t H[7*7];
	real_t A[50*7];
	real_t g[7];
	real_t lbA[50];

	/*	    ( 1.0 0.5 |                    )
	 *	    ( 0.5 2.0 |                    )
	 *	    ( --------+------------------- )
	 *	H = (         | 1e-6               )
	 *	    (         |      1e-6          )
	 *	    (         |           ...      )
	 *	    (         |               1e-6 ) */
	for( i=0; i<7*7; ++i )
		H[i] = 0.0;
	for( i=2; i<7; ++i )
		H[i*7+i] = 1.0e-6;
	H[0] = 1.0;
	H[1] = 0.5;
	H[7] = 0.5;
	H[8] = 2.0;

	/*	    ( x.x x.x | 1.0             )
	 *	    ( x.x x.x | ...             )
	 *	    ( x.x x.x | 1.0             )
	 *	    ( x.x x.x |     1.0         )
	 *	A = ( x.x x.x |     ...         )
	 *	    ( x.x x.x |     1.0         )
	 *	    ( x.x x.x |         ...     )
	 *	    ( x.x x.x |             1.0 )
	 *	    ( x.x x.x |             ... )
	 *	    ( x.x x.x |             1.0 ) */
	for( i=0; i<50*7; ++i )
		A[i] = 0.0;
	for( i=0; i<50; ++i )
	{
		for( j=0; j<2; ++j )
			A[i*7+j] = (real_t)rand() / (real_t)RAND_MAX;

		A[i*7 + (i/10)+2] = 1.0;
	}

	/*	    ( -1.0 )
	 *	    ( -0.5 )
	 *	    ( ---- )
	 *	g = (      )
	 *	    (      )
	 *	    (      )
	 *	    (      ) */
	for( i=0; i<7; ++i )
		g[i] = 0.0;
	g[0] = -1.0;
	g[1] = -0.5;

	for( i=0; i<50; ++i )
		lbA[i] = 1.0;

	/* ... and setting up user-defined constraint product function. */
	MyConstraintProduct myCP( 7,50,A );


	/* Setting up QProblem object and set construct product function. */
	QProblem exampleCP( 7,50 );
	exampleCP.setPrintLevel( PL_NONE );
	
	exampleCP.setConstraintProduct( &myCP );


	/* Solve first QP. */
	real_t cputime = 1.0;
	int_t nWSR = 100;
	exampleCP.init( H,g,A,0,0,lbA,0, nWSR,&cputime );


	/* Solve second QP using a modified gradient. */
	g[0] = -2.0;
	g[1] =  0.5;

	cputime = 1.0;
	nWSR = 100;
	exampleCP.hotstart( g,0,0,lbA,0, nWSR,&cputime );

	/* Get and print solution of second QP. */
	real_t xOpt[7];
	exampleCP.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e, %e ... ];  objVal = %e\n", xOpt[0],xOpt[1],xOpt[2],exampleCP.getObjVal() );
	printf( "CPU time:  %.3f microseconds\n\n", cputime*1.0e6 ); 

	
	
	/* Do the same without specifying constraint product. */
	QProblem example( 7,50 );
	example.setPrintLevel( PL_NONE );
	
	/* Solve first QP. */
	g[0] = -1.0;
	g[1] = -0.5;
	
	cputime = 1.0;
	nWSR = 100;
	example.init( H,g,A,0,0,lbA,0, nWSR,&cputime );
	
	/* Solve second QP using a modified gradient. */
	g[0] = -2.0;
	g[1] =  0.5;

	cputime = 1.0;
	nWSR = 100;
	example.hotstart( g,0,0,lbA,0, nWSR,&cputime );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e, %e ... ];  objVal = %e\n", xOpt[0],xOpt[1],xOpt[2],example.getObjVal() );
	printf( "CPU time:  %.3f microseconds\n\n", cputime*1.0e6 ); 
	
	return 0;
}


/*
 *	end of file
 */

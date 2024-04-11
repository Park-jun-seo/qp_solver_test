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
 *	\file examples/exampleLP.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.2
 *	\date 2008-2017
 *
 *	Very simple example for solving a LP sequence using qpOASES.
 */



#include <qpOASES.hpp>


/** Example for qpOASES main function solving LPs. */
int main( )
{
	USING_NAMESPACE_QPOASES

	/* Setup data of first LP. */

	real_t A[2*2] = { 1.0, 1.0 ,
					  2.0/3.0, 3};
	real_t g[2] = { -5.0/3.0, -1.0 };
	real_t lb[2] = { 0.0 , 0.0 };
	real_t ub[2] = { INFTY, INFTY };
	real_t lbA[2] = { -INFTY, -INFTY };
	real_t ubA[2] = { 1.0, 1.0};

	/* Setting up QProblem object with zero Hessian matrix. */
	QProblem example( 2,2,HST_ZERO );

	Options options;
    // options.printLevel = PL_NONE; // 로그 출력 비활성화
 	//options.setToMPC();
	example.setOptions( options );

	/* Solve first LP. */
	int_t nWSR = 100;
	example.init( 0,g,A,lb,ub,lbA,ubA, nWSR,0 );

	/* Get and print solution of second LP. */
	real_t xOpt[2];
	example.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );

	return 0;
}


/*
 *	end of file
 */
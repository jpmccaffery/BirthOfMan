/*
 * Copyright (C) 2007 Derek James 
 * 
 * This file is part of SIPHON (Simulating the Phylogeny and Ontogeny of the Neocortex).
 * 
 * SIPHON is free software; you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with this program; if
 * not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307 USA
 * 
 * created by Derek James on November 4th, 2007
 */
package network;

/**
 * Rule for updating connections weights due to spike timing dependent plasticity
 * (STDP). First, the difference between the postsynaptic firing time and the 
 * presynaptic firing time is calculated. If the difference is positive, the weight
 * is increased using the alpha function of the form f(x) = x(exp(-x), analogous to
 * inducing long-term potentiation (LTP). If the difference is negative, the weight
 * is decreased using the same function, simply changing the sign, analagous to 
 * inducing long-term depression (LTD).
 * 
 * @author Derek James
 */

public class STDPFunction {

private double weightDelta;
private double timeDiff;
private double learningRate = 1.0;
private int learningWindow = 10;

public double calcWeightChange(double presynFiringTime, double postsynFiringTime, double currentWeight) {	
	
	weightDelta = 0.0;
	timeDiff = ( postsynFiringTime - presynFiringTime );
	//System.out.println(timeDiff);
	if ( Math.abs(timeDiff) < learningWindow ) {
		//System.out.println("weight updated");
		if ( 0 < timeDiff ) {
			weightDelta = learningRate * ( timeDiff * (Math.exp( -timeDiff )) ); //alpha function
			weightDelta = weightDelta / (1 + (currentWeight * currentWeight));   //bounds the weight change
		} else if ( timeDiff < 0 ) {
			timeDiff = Math.abs(timeDiff);
			weightDelta = - learningRate * ( timeDiff * (Math.exp( -timeDiff )) ); //negative of alpha function
			weightDelta = weightDelta / (1 + (currentWeight * currentWeight));     //bounds the weight change
		}
	}
	return weightDelta;
}

}

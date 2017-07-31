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
 * Connection is a component of an artificial neural network,
 * connecting two neurons.
 * They conform to Dale's Principle, so if a connection's presynaptic
 * neuron is excitatory, then it has a positive weight value. If the
 * connection's presynaptic neuron is inhibitory, then it has a negative
 * weight value.
 * A connection may also be a delay connection, so that it holds the
 * transmission of its signal for a given amount of time (usually 1
 * time step).
 *
 * @author Derek James
 */

public class Connection {

private long id;
private long presynapticId;
private long postsynapticId;
private int delay = 0;

//Source and destination locations
//private float srcX, srcY, srcZ, destX, destY, destZ;
private float[] srcAndDestLocs = new float[6];

private double weight = 0.0d;
private double maxWeightMagnitude = 0.5d; //Maximum absolute value for any connection weight
private boolean saturated = false;
private boolean LTPJustInduced = false;
private double lastDeltaW = 0.0d;

public boolean getLTPJustInduced() {
	return LTPJustInduced;
}

public void setLTPJustInduced(boolean newLTPJustInducedSetting) {
	LTPJustInduced = newLTPJustInducedSetting;
}

public void setWeight (double newWeight) {
	//If the weight has already been saturated to its max, then no weight update
	//occurs, resulting in non-reversible learning upon saturation
	if (!saturated) {
		//Bounds the weights by the max magnitude allowed
		if (newWeight < -maxWeightMagnitude) {
			newWeight = -maxWeightMagnitude;
			saturated = true;
		} else if (newWeight > maxWeightMagnitude) {
			newWeight = maxWeightMagnitude;
			saturated = true;
		}
			weight = newWeight;
	}
}

public double getLastDeltaW () {
	return lastDeltaW;
}

public void setLastDeltaW (double newLastDeltaW) {
	lastDeltaW = newLastDeltaW;
}


public boolean getSaturated () {
	return saturated;
}

public void setSaturated (boolean newSaturatedSetting) {
	saturated = newSaturatedSetting;
}

public double getWeight () {
	return weight;
}

public void setId (long newId) {
	id = newId;
}

public long getId () {
	return id;
}

public void setPresynapticId (long newPresynapticId) {
	presynapticId = newPresynapticId;
}

public void setPostsynapticId (long newPostsynapticId) {
	postsynapticId = newPostsynapticId;
}

public long getPresynapticId () {
	return presynapticId;
}

public long getPostsynapticId () {
	return postsynapticId;
}

public float[] getSrcAndDestLocation() {
	return srcAndDestLocs;
}

public void setSrcAndDestLocation(float newSrcX, float newSrcY, float newSrcZ,
		float newDestX, float newDestY, float newDestZ) {
	srcAndDestLocs[0] = newSrcX;
	srcAndDestLocs[1] = newSrcY;
	srcAndDestLocs[2] = newSrcZ;
	srcAndDestLocs[3] = newDestX;
	srcAndDestLocs[4] = newDestY;
	srcAndDestLocs[5] = newDestZ;
}

public void setDelay (int newDelay) {
	delay = newDelay;
}

public int getDelay() {
	return delay;
}

}

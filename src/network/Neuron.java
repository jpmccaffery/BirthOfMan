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

import java.util.List;


/**
 * Implements an abstract interface for neurons.
 *
 * @author Derek James
 */

public interface Neuron {

public void setId (long newId );

public long getId();

//only used for delay neurons
public void setParentId (long newParentId );
public long getParentId ();
public void setDelay (int newDelay );
public int getDelay ();

public void reset();

public int getExceededThreshold ();
public int getJustFired();
public void setJustFired(int newJustFired );

public double getMostRecentFiringTime();

public void setType(String newType );

public String getType();

public void setLocation(float newX, float newY, float newZ );

public float[] getLocation();

public void setExcitatory(boolean newExcitatorySetting );

public boolean getExcitatory();

public double getMembranePotential();

public void setCurrentTime(double newCurrentTime );

public void setMembranePotential(double membranePotentialToSet );

public double getThreshold();

public List<Double> getPreviousFiringTimes ();

public void setPreviousFiringTimes (List<Double> newPreviousFiringTimes );

public void inhibitNeuron();

public double getSynapticOutput();

public void setSynapticOutput(double newSynapticOutput );

public double getPreviousMembranePotential (int delay );

public double getPreviousActivationLevel (int delay );

public List<Connection> getAfferentConnections ();


//Given input, updates the membrane potential of the neuron.
public void updateMembranePotential (double input, double currentTime );

public void synchronizeMembranePotential ();

public void updateAfferentConnections (List<Connection> connections);

public void updateEfferentConnections (List<Connection> connections);

public void setTimeConstant (double newTimeConstant );

public void setDeltaT (double newDeltaT );

public boolean getActiveDuringLearningWindow();

public void setActiveDuringLearningWindow(boolean newActiveDuringLearningWindow);

public void setRecentLTPMax(double newRecentLTPMax);

public double getRecentLTPMax();

public void setNeighborActivation(double newNeighborActivation);

}

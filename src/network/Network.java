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

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.TreeSet;

public class Network
{

public long id;
private double timeConstant;
private double deltaTime;
private double currentTime;
private int currentStep;
private double noise;
private String type;
private boolean winnerTakeAll;

private static List<Neuron> neurons;
private static List<Connection> connections;
private static List<Neuron> inputNeurons;
private static List<Neuron> outputNeurons;

private double membranePotentialToGet;
private double synapticOutputToGet;
private double inhibitedMembranePotential;
private double heterosynapticLTDPercentage;

private static double learningRate = 1.0;
private static int learningDelay = 1;
private static int learningWindow;
private double standardDev = 0.30;
private double mean = 0;
private List<Connection> connsToAdd = new ArrayList<Connection> ();

public Network ()
{
	id = 0;
	timeConstant = 10.0;
	deltaTime = 0.02;
	currentTime = 0;
	currentStep = 0;
	noise = 0.0;
	type = "leakyIntegrateAndFire";
	neurons = new ArrayList<Neuron> ();
	connections = new ArrayList<Connection> ();
	inputNeurons = new ArrayList<Neuron> ();
	outputNeurons = new ArrayList<Neuron> ();
	winnerTakeAll = false;
	inhibitedMembranePotential =  0.0;
	heterosynapticLTDPercentage = 0.5;
}


public Collection<Neuron> getNeuronList ()
{
	//System.out.println (neurons.size ());
	return neurons;
}

public Collection<Connection> getConnectionList ()
{
	//System.out.println (connections.size ());
	return connections;
}

public void setId (long newId)
{
	id = newId;
}

public long getId ()
{
	return id;
}

public void setTimeConstant (double newTimeConstant)
{
	timeConstant = newTimeConstant;
}

public double getTimeConstant ()
{
	return timeConstant;
}

public void setDeltaTime (double newDeltaTime)
{
	deltaTime = newDeltaTime;
}

public double getDeltaTime ()
{
	return deltaTime;
}

public void setType (String newType)
{
	type = newType;
}

public String getType ()
{
	return type;
}

public double getCurrentTime ()
{
	return currentTime;
}

public void setWinnerTakeAll (boolean newWinnerTakeAllSetting)
{
	winnerTakeAll = newWinnerTakeAllSetting;
}

public void initWeights  (double standardDev, double mean)
{
	Iterator connIterator = connections.iterator ();
	Random rand = new Random ();
	while  (connIterator.hasNext ())
{
		Connection c =  (Connection) connIterator.next ();
		c.setSaturated (false);
		double newWeight = ((standardDev * rand.nextGaussian()) + mean);
		c.setWeight(newWeight);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////

public void updateWeightsViaSTDP()
{
		//update this method to make it more efficient by iterating through
		//neurons first to check if they've fired recently
	Iterator connIterator = connections.iterator();
	STDPFunction stdp = new STDPFunction();
	while (connIterator.hasNext())
{
		double deltaW = 0.0;
		Connection c = (Connection) connIterator.next();
		double presynFiringTime = getMostRecentFiringTimeByID(c.getPresynapticId());
		double postsynFiringTime = getMostRecentFiringTimeByID(c.getPostsynapticId());
		//logic here to make sure STDP only gets updated once per firing
		double mostRecentFiringTime = 0.0;
		if (presynFiringTime > postsynFiringTime)
{
			mostRecentFiringTime = presynFiringTime;
		} else
{
			mostRecentFiringTime = postsynFiringTime;
		}
		double roundedCurrentTime = (currentTime - deltaTime);
		long long1 = (int)Math.round(roundedCurrentTime * 100);
		roundedCurrentTime = long1 / 100.0;
		double roundedMostRecentFiringTime = mostRecentFiringTime;
		long long2 = (int)Math.round(roundedMostRecentFiringTime * 100);
		roundedMostRecentFiringTime = long2 / 100.0;
		if ((presynFiringTime > 0) && (postsynFiringTime > 0) &&
				(roundedCurrentTime == roundedMostRecentFiringTime))
{
			double currentWeight = c.getWeight();
			deltaW = stdp.calcWeightChange(presynFiringTime, postsynFiringTime, currentWeight);
			double oldWeight = c.getWeight();
			double newWeight = oldWeight + deltaW;
			c.setWeight(newWeight);
			if (oldWeight < newWeight)
{
				c.setLTPJustInduced(true);
				//System.out.println("LTP induced");
			}

		}
		//System.out.println(c.getWeight());
	}
}

///////////////////////////////////////////////////////////////////////////////////////////

/* Implements a learning rule which approximates STDP with non-spiking neurons.
 * CTAHL stands for "Continuous Temporally Asymmetric Hebbian Learning"
 * See Dayan & Abbott's "Theoretical Neuroscience" (2001) Equation 8.18 and
 * brief discussion for reference.
 */
public void updateWeightsViaCTAHL (int lWindow)
{
	learningWindow = lWindow;
	Iterator connIterator = connections.iterator();
	while (connIterator.hasNext())
{
			double deltaW = 0.0;
			//Efficiency check: iterate through all the connections, and first
			//check that both pre- and post-synaptic neurons had non-zero
			//activity during the previous learning window; if not, then don't
			//bother applying the learning rule to the connection
			Connection c = (Connection) connIterator.next();
			long preSynId = c.getPresynapticId();
			long postSynId = c.getPostsynapticId();
			if (checkIfActiveDuringLearningWindow(preSynId) &&
				checkIfActiveDuringLearningWindow(preSynId))
{
				//apply learning rule
				double[] preSynActivation = getNeuronActivationForWindow(preSynId, learningWindow - 1);
				double[] postSynActivation = getNeuronActivationForWindow(postSynId, learningWindow - 1);							  
				for (int i = learningDelay; i < (learningWindow - learningDelay); i++)
{
				   deltaW += (learningRate * (postSynActivation[i] * preSynActivation[i - learningDelay]))
							- (learningRate * (preSynActivation[i] * postSynActivation[i - learningDelay]));
				}
				double oldWeight = c.getWeight();
				if (deltaW < 0)
{
					deltaW = 0;
				}
				double newWeight = oldWeight + deltaW;
				c.setWeight(newWeight);
				//TODO:
				if (deltaW > 0)
{
					System.out.println("LTP induced on " + c.getId());
					System.out.println("Weight changed by " + (deltaW));
					System.out.println("Old weight: " + oldWeight);
					System.out.println("New weight: " + c.getWeight());
					System.out.println();
				}

				//Records the max LTP induced relative to the postsynaptic neuron
				//Used for heterosynaptic LTD
				Iterator neuronIt = neurons.iterator();
				while (neuronIt.hasNext())
{
					Neuron n = (Neuron) neuronIt.next();
					if (n.getId() == c.getPostsynapticId())
{
						double tempLTPMax = n.getRecentLTPMax();
						if (deltaW > tempLTPMax)
{
							n.setRecentLTPMax(deltaW);
						}
					}
				}

				if (oldWeight < newWeight)
{
				   c.setLTPJustInduced(true);
				   //System.out.println("LTP induced");
				}
			}
	}
	//After applying any learning, reset the flag indicating
	//that the neurons were active during the learning window
	Iterator neuronIt = neurons.iterator();
	while (neuronIt.hasNext())
{
			Neuron n = (Neuron) neuronIt.next();
			n.setActiveDuringLearningWindow(false);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////


/* This method induces heterosynaptic LTD, which is a phenomenon where, when LTP
 * is induced on at least one presynaptic synapse, all presynaptic synapses that
 * were not increased are then decreased.
 */
public void induceHeterosynapticLTD ()
{
	Iterator<Neuron> neuronIt = neurons.iterator ();

	while (neuronIt.hasNext ())
	{
		boolean presynapticLTPInduced = false;
		Neuron n = neuronIt.next ();

		List<Connection> conns = n.getAfferentConnections ();
		Iterator<Connection> connIt = conns.iterator ();
		//Checks to see if at least one presynaptic connection
		//was increased via STDP
		while (connIt.hasNext ())
		{
			Connection c = connIt.next ();

			if (c.getLTPJustInduced ())
			{
				presynapticLTPInduced = true;
			}
		}
		//Iterates through presynaptic connections again, and if LTP was
		//induced on at least one, all others are decreased by a certain percentage
		Iterator<Connection> connIt2 = conns.iterator ();

		while (connIt2.hasNext ())
		{
			Connection c = connIt2.next();
			double inducedLTD = 0.0;
			double tempWeight = c.getWeight();
			double oldWeight = tempWeight;
			double newWeight = 0.0;

			if (presynapticLTPInduced && !c.getLTPJustInduced ())
			{
				System.out.println ("LTD induced");
				if (type.equals ("leakyIntegrateAndFire"))
				{
					tempWeight = (tempWeight - (tempWeight * heterosynapticLTDPercentage));
					c.setWeight (tempWeight);
				}
				if (type.equals ("leakyIntegrator"))
				{
					inducedLTD = getNeuronByID (c.getPostsynapticId ()).getRecentLTPMax ();
					if (inducedLTD < 0.1)
					{
						inducedLTD = 0;
					}
					newWeight = oldWeight - inducedLTD;
					c.setWeight(newWeight);
				}
			}
			if (inducedLTD > 0)
			{
				System.out.println ("LTD induced on " + c.getId ());
				System.out.println ("Weight changed by -" + inducedLTD);
				System.out.println ("Old weight: " + oldWeight);
				System.out.println ("New weight: " + newWeight);
				System.out.println ();
			}
		}
		//Reset LTPJustInduced on all connections to false
		Iterator<Connection> connIt3 = conns.iterator ();

		while (connIt3.hasNext ())
		{
			Connection c = connIt3.next ();
			c.setLTPJustInduced (false);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////

public Neuron getNeuronByID (long neuronID)
{
	Neuron neuronToGet = null;
	Iterator<Neuron> neuronIt = neurons.iterator ();

	while (neuronIt.hasNext ())
	{
			Neuron n = neuronIt.next ();

			if (n.getId () == neuronID)
			{
				neuronToGet = n;
			}
	}

	return neuronToGet;
}


///////////////////////////////////////////////////////////////////////////////////////////


public double getMostRecentFiringTimeByID (long neuronID)
{
	double mostRecentFiringTime = 0.0;
	Iterator neuronIt = neurons.iterator();
	while (neuronIt.hasNext())
{
		Neuron n = (Neuron) neuronIt.next();
		if (n.getId() == neuronID)
{
			mostRecentFiringTime = n.getMostRecentFiringTime();
		}
	}
	return mostRecentFiringTime;
}

private boolean checkIfActiveDuringLearningWindow (long neuronID)
{
	boolean wasActive = false;
	Iterator neuronIt = neurons.iterator();
	while (neuronIt.hasNext())
{
		Neuron n = (Neuron) neuronIt.next();
		if (n.getId() == neuronID)
{
					wasActive = n.getActiveDuringLearningWindow();
				}
	}
	return wasActive;
}



private double[] getNeuronActivationForWindow(long neuronId, int lWindow)
{
	double[] neuronActivation = new double[lWindow];
	Iterator neuronIt = neurons.iterator();
	while (neuronIt.hasNext())
{
			Neuron n = (Neuron) neuronIt.next();
			if (n.getId() == neuronId)
{
				for (int i = 0; i < (lWindow - 1); i++)
{
					neuronActivation[i] = n.getPreviousActivationLevel(i + currentStep - lWindow);				   
				}
			}
	}
	return neuronActivation;
}

public void reset()
{
	currentTime = 0;
	currentStep = 1;
	Iterator iter = neurons.iterator();
	while (iter.hasNext())
{
		Neuron n = (Neuron) iter.next();
		n.reset();
		//System.out.println(n.getMembranePotential());
	}
}

/* Given input, fully activates the network based on timing parameters
 * and returns output.
 */
public void activate (double[] input)
{
	//Temporarily updates membrane potentials for all input neurons
	//This is done first because the total input to input neurons is
	//a function of external input plus input from other neurons.
	Iterator<Neuron> inputNeuronIt = inputNeurons.iterator();
	int counter = 0;

	while (inputNeuronIt.hasNext ())
	{
		Neuron n = inputNeuronIt.next ();
		double totalInput = 0.0;
		totalInput += input[counter];
		List<Connection> inboundConns = n.getAfferentConnections ();
		Iterator<Connection> connIterator = inboundConns.iterator ();
		while (connIterator.hasNext ())
		{
			Connection c = connIterator.next ();
			double preSynapticActivation =
				getNeuronActionPotentialByID (c.getPresynapticId ());
			double inputFromThisConn = (c.getWeight () * preSynapticActivation);
			totalInput += inputFromThisConn;
		}
		n.updateMembranePotential (totalInput, currentTime);
		counter++;
	}

	//Temporarily updates membrane potentials for all output and hidden neurons
	Iterator<Neuron> neuronIt2 = neurons.iterator();

	while (neuronIt2.hasNext())
	{
		Neuron n = neuronIt2.next ();

		if (n.getType ().equals ("output") || n.getType ().equals ("hidden"))
		{
			double totalInput = 0.0;
			List<Connection> inboundConns = n.getAfferentConnections ();
			Iterator<Connection> connIterator = inboundConns.iterator ();

			while (connIterator.hasNext ())
			{
				Connection c = connIterator.next ();
				double preSynapticActivation = 0.0;
				preSynapticActivation += getNeuronActionPotentialByID (c.getPresynapticId ());
				double inputFromThisConn = (c.getWeight () * preSynapticActivation);
				totalInput += inputFromThisConn;
			}
			n.updateMembranePotential (totalInput, currentTime);
		}
	}

	//Update all delay neurons
	Iterator<Neuron> neuronIt4 = neurons.iterator();
	while (neuronIt4.hasNext())
	{
		Neuron n = neuronIt4.next ();
		if (n.getType ().toString ().equals ("delay"))
		{
			long thisParentId = n.getParentId ();
			int thisDelaySize = n.getDelay ();
			int thisDelayIdx = (currentStep - thisDelaySize);

			if (thisDelaySize < currentStep)
			{
				Iterator<Neuron> neuronIt5 = neurons.iterator ();

				while (neuronIt5.hasNext ())
				{
					Neuron n2 = neuronIt5.next ();

					if (n2.getId () == thisParentId)
					{
						double delayedMembranePotential =
							n2.getPreviousMembranePotential (thisDelayIdx - 1);

						n.setCurrentTime (currentTime);
						n.setMembranePotential (delayedMembranePotential);
					}
				}
			}
		}
	}

	//If winner-take-all is in effect, reduce all membrane potentials other than the
	//highest for a given layer
	if (winnerTakeAll)
	{
		updateViaWinnerTakeAll();
	}

	//Makes all the temporary updates to membrane potentials permanent,
	//insuring synchronous update.
	Iterator<Neuron> neuronIt3 = neurons.iterator();
	while (neuronIt3.hasNext ())
	{
			Neuron n = neuronIt3.next();
			n.synchronizeMembranePotential();
	}

	currentStep++;
	currentTime += deltaTime;
}

public double getNeuronMembranePotentialByID (long preSynapticID)
{
	Iterator neuronIt = neurons.iterator();
	while (neuronIt.hasNext())
{
		Neuron n = (Neuron) neuronIt.next();
		if (n.getId() == preSynapticID)
{
			membranePotentialToGet = n.getMembranePotential();
		}
	}
	return membranePotentialToGet;
}

public double getNeuronActionPotentialByID (long preSynapticID)
{
	Iterator neuronIt = neurons.iterator();
	while (neuronIt.hasNext())
{
		Neuron n = (Neuron) neuronIt.next();
		if (n.getId() == preSynapticID)
{
			synapticOutputToGet = n.getSynapticOutput();
		}
	}
	return synapticOutputToGet;
}

public void addNeuron(Neuron newNeuron)
{
	neurons.add(newNeuron);
	if (newNeuron.getType().equals("input"))
{
		inputNeurons.add(newNeuron);
	}
	if (newNeuron.getType().equals("output"))
{
		outputNeurons.add(newNeuron);
	}
}

public void addConnection(Connection newConnection)
{
	connections.add(newConnection);
}


public boolean checkIfConnected(long neuron1id, long neuron2id)
{
	boolean areConnected = false;
		Iterator connIterator = connections.iterator();
	while (connIterator.hasNext())
{
		 Connection c = (Connection) connIterator.next();
			 if (c.getPresynapticId() == neuron1id && c.getPostsynapticId() == neuron2id)
{
				 areConnected = true;
			 }
	}

	return areConnected;
}

public void connectNeurons(long neuron1id, long neuron2id)
{
	Connection c = new Connection();
	Connection tempC = (Connection) connections.get(connections.size()-1);
	long newConnId = tempC.getId()+1;
	c.setId(newConnId);
	c.setPresynapticId(neuron1id);
	c.setPostsynapticId(neuron2id);
	Neuron n1 = getNeuronByID(neuron1id);
	Neuron n2 = getNeuronByID(neuron2id);
	float[] n1location = n1.getLocation();
	float[] n2location = n2.getLocation();
	c.setSrcAndDestLocation(n1location[0], n1location[1], n1location[2], n2location[0], n2location[1], n2location[2]);
	Random rand = new Random();
	double newWeight = ((standardDev * rand.nextGaussian()) + mean);
	c.setWeight(newWeight);
	connections.add(c);
}



public void mutateNeuron(long connectionId)
{
	Iterator connIterator = connections.iterator();
	while (connIterator.hasNext())
{
		Connection c = (Connection) connIterator.next();
		if (c.getId() == connectionId)
{
			long presynId = c.getPresynapticId();
			long postsynId = c.getPostsynapticId();
			double oldWeight = c.getWeight();
			Neuron n1 = getNeuronByID(presynId);
			Neuron n2 = getNeuronByID(postsynId);
			float[] n1location = n1.getLocation();
			float[] n2location = n2.getLocation();
			Neuron n3 = new LeakyIntegratorNeuron();
			Neuron tempN = (Neuron) neurons.get(neurons.size()-1);
			long newNeuronId = tempN.getId()+1;
			n3.setId(newNeuronId);
			float[] n3location = new float[3];
			n3location[0] = (n1location[0] + n2location[0]) / 2;
			n3location[1] = (n1location[1] + n2location[1]) / 2;
			n3location[2] = n1location[2] + n2location[2] + 10;
			n3.setLocation(n3location[0], n3location[1], n3location[2]);
			n3.setType("hidden");
			neurons.add(n3);
			Connection newPresynConn = new Connection();
			Connection tempC = (Connection) connections.get(connections.size()-1);
			long newConnId = tempC.getId()+1;
			newPresynConn.setId(newConnId);
			newPresynConn.setPresynapticId(n1.getId());
			newPresynConn.setPostsynapticId(n3.getId());
			newPresynConn.setSrcAndDestLocation(n1location[0], n1location[1], n1location[2], n3location[0], n3location[1], n3location[2]);
			newPresynConn.setWeight(oldWeight);
			Connection newPostsynConn = new Connection();
			newPostsynConn.setId(newConnId+1);
			newPostsynConn.setPresynapticId(n3.getId());
			newPostsynConn.setPostsynapticId(n2.getId());
			newPostsynConn.setSrcAndDestLocation(n3location[0], n3location[1], n3location[2], n2location[0], n2location[1], n2location[2]);
			newPostsynConn.setWeight(1.0);
			connsToAdd.add(newPresynConn);
			connsToAdd.add(newPostsynConn);
		}
	}
}

public void clearConnsToAdd()
{
	connsToAdd.clear();
}

public void addBatchConnections()
{
	connections.addAll(connsToAdd);
}

public void updateAllConnectivity ()
{
	Iterator neuronIt = neurons.iterator();
	while (neuronIt.hasNext())
{
		Neuron n = (Neuron) neuronIt.next();
		n.updateAfferentConnections (connections);
		n.updateEfferentConnections (connections);
	}
}


public String getAllSpikeTrains()
{
	StringBuffer sb = new StringBuffer();
	Iterator neuronIt = neurons.iterator();
	while (neuronIt.hasNext())
{
		Neuron n = (Neuron) neuronIt.next();
		sb.append(n.getPreviousFiringTimes().toString());
		sb.append("\n");
	}
	return sb.toString();
}

/* This method is only used if the winner-take-all option is selected.
 * Neurons are temporarily grouped by their common y-position, and the
 * neuron that just fired has its membrane potential set to zero, while
 * its neighbors are set to "inhibited" for a certain amount of time, during
 * which they receive negative input.
 */
private void updateViaWinnerTakeAll ()
{
	float currentY = -1.0f;
	double exp = 2.0;
	double max = 1.0;
	double midpointToMax = 0.5;
	Set yValues = new TreeSet();
	int neuronCounter = 0;
	Iterator neuronIt5 = neurons.iterator();
	while (neuronIt5.hasNext ())
	{
		Neuron n = (Neuron) neuronIt5.next();
		//if (n.getType().equals("output") || n.getType().equals("hidden") || n.getType().equals("input"))
		if (n.getType().equals("output") || n.getType().equals("hidden"))
		{
			yValues.add(n.getLocation()[1]);
			//System.out.println(n.getLocation()[1]);
			neuronCounter++;
		}
	}

	while (neuronCounter > 0)
	{
		Iterator yValuesIt = yValues.iterator();
		boolean gotNewMax = false;
		while (yValuesIt.hasNext() && !gotNewMax)
		{
			Float newY = (Float) yValuesIt.next();
			if (newY > currentY)
			{
				currentY = newY;
				gotNewMax = true;
			}
		}
		//System.out.println("Y: " + currentY);
		List<Neuron> neuronsInSameLayer = new ArrayList<Neuron> ();
		Iterator<Neuron> neuronIt = neurons.iterator ();

		while (neuronIt.hasNext ())
		{
			Neuron n = neuronIt.next();
			//if ((n.getLocation()[1] == currentY) && (n.getType().equals("output") || n.getType().equals("hidden") || n.getType().equals("input")) )
			if ((n.getLocation ()[1] == currentY) &&
			    (n.getType ().equals ("output") || n.getType ().equals ("hidden")) )
			{
				neuronsInSameLayer.add (n);
			}
		}

		List<Neuron> neuronThatJustFired = new ArrayList<Neuron> ();

		if (type.equals ("leakyIntegrateAndFire"))
		{
			//If two neurons at the same level have a membrane potential
			//that exceeds the threshold, only the first is selected and allowed
			//to fire
			Iterator<Neuron> neuronIt2 = neuronsInSameLayer.iterator ();
			while (neuronIt2.hasNext ())
			{
					Neuron n = neuronIt2.next ();
					if (n.getExceededThreshold () == 1)
					{
							neuronThatJustFired.clear ();
							neuronThatJustFired.add (n);
					}
			}
			//inhibits all other neurons than the one that was allowed to fire
			if (! neuronThatJustFired.isEmpty ())
			{
				neuronsInSameLayer.remove(neuronThatJustFired.get (0));
				Iterator<Neuron> neuronIt3 = neuronsInSameLayer.iterator ();

				while (neuronIt3.hasNext ())
				{
						Neuron n = neuronIt3.next ();

						n.inhibitNeuron ();
						neuronCounter--;
				}
			}
		}
		if (type.equals ("leakyIntegrator"))
		{
			double totalLayerActivation = 0.0;
			//double betaParam = 2.5;
			Iterator<Neuron> neuronIt2 = neuronsInSameLayer.iterator ();

			while (neuronIt2.hasNext ())
			{
				Neuron n = neuronIt2.next();
				double currentNeuronActivation = n.getMembranePotential ();

				if (currentNeuronActivation >= 0)
				{
					totalLayerActivation += (max * Math.pow (currentNeuronActivation, exp)) /
					                        (Math.pow (midpointToMax, exp) + Math.pow (currentNeuronActivation, exp));
				}
			}

			Iterator<Neuron> neuronIt3 = neuronsInSameLayer.iterator();

			while (neuronIt3.hasNext ())
			{
				Neuron n = neuronIt3.next ();
				double currentNeuronActivation = n.getMembranePotential ();
				double currentNeuronTransformedActivation = 0.0;

				if (currentNeuronActivation >= 0)
				{
					currentNeuronTransformedActivation = (max * Math.pow (currentNeuronActivation, exp)) /
					                                     (Math.pow (midpointToMax, exp) + Math.pow (currentNeuronActivation, exp));
				}

				double neighborActivation = totalLayerActivation - currentNeuronTransformedActivation;
				n.setNeighborActivation (neighborActivation);
			}
		}
		neuronCounter--;
	}
}

public List<Neuron> getInputNeurons ()
{
	return inputNeurons;
}

public List<Neuron> getOutputNeurons ()
{
	return outputNeurons;
}

}

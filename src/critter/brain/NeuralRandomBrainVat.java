package critter.brain;

import evolver.NetworkCreator;

import network.Network;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;


public class NeuralRandomBrainVat extends NeuralBrainVat
{
	public NeuralRandomBrainVat (String saveName_, int index_, Class type_)
	{
		super (type_);

		m_saveName = saveName_;
		m_index = index_;
	}

	public NeuralRandomBrainVat (String saveName_, int index_)
	{
		this (saveName_, index_, NeuralStraightBrain.class);
	}

	public Network getNetwork (int numInputs_, int numOutputs_)
	{
		try
		{
			File saveFile = new File(m_saveName);
			BufferedWriter bwOut = new BufferedWriter(new FileWriter(saveFile));

			NetworkCreator nc = new NetworkCreator ();
			String netXMLString = nc.createNetwork (m_index, numInputs_, numOutputs_);

			bwOut.write(netXMLString);
			bwOut.close();
		}
		catch (IOException exception_)
		{
			System.err.println ("NeuralRandomBrainVat::getNetwork:");
			System.err.println (exception_.getMessage ());
			System.exit (0);
		}

		NeuralXMLBrainVat vat = new NeuralXMLBrainVat (m_saveName);

		return vat.getNetwork (numInputs_, numOutputs_);
	}

	private final int m_index;
	private final String m_saveName;
}

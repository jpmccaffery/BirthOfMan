package critter.brain;

import network.Network;
import network.NetworkBuilder;

import java.io.File;


public class NeuralXMLBrainVat extends NeuralBrainVat
{
	public NeuralXMLBrainVat (String fileName_, Class type_)
	{
		super (type_);

		m_fileName = fileName_;
	}

	public NeuralXMLBrainVat (String fileName_)
	{
		this (fileName_, NeuralStraightBrain.class);
	}

	public Network getNetwork (int numInputs_, int numOutputs_)
	{
		File file = new File (m_fileName);
		NetworkBuilder nnBuilder = new NetworkBuilder ();
		Network nn = nnBuilder.buildNetworkFromFile (file);

		nn.setWinnerTakeAll (true);

		return nn;
	}

	private final String m_fileName;
}

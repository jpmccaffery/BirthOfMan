package critter.brain;

import network.Network;
import network.Neuron;

import java.util.ArrayList;
import java.util.List;


public class NeuralStraightBrain implements Brain
{
	public NeuralStraightBrain (Network network_)
	{
		m_network = network_;
	}

	public List<Float> readOutput (float tpf_)
	{
		List<Float> output = new ArrayList<Float> ();
		List<Neuron> oNeurons = m_network.getOutputNeurons ();

		for (Neuron n : oNeurons)
		{
			output.add ((float) n.getMembranePotential ());
		}

		return output;
	}

	public void activate (List<Float> input_, float tpf_)
	{
		if (input_.size () != m_network.getInputNeurons ().size ())
		{
			System.out.println ("PushPullBrain::activate: Size mismatch");
			System.exit (0);
		}

		double[] dInputs = new double[input_.size ()];

		for (int i = 0; i < input_.size (); i++)
			dInputs[i] = (double) input_.get (i);

		m_network.activate (dInputs);
	}

	public static int nnInputs (int inputs_)
	{
		return inputs_;
	}

	public static int nnOutputs (int outputs_)
	{
		return outputs_;
	}

	private final Network m_network;
}

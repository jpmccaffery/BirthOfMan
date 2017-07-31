package critter.brain;

import network.Network;
import network.Neuron;

import java.util.ArrayList;
import java.util.List;


public class NeuralPushPullBrain implements Brain
{
	public NeuralPushPullBrain (Network network_)
	{
		if (network_.getOutputNeurons ().size () % 2 != 0)
		{
			System.err.println ("NeuralPushPullBrain::NeuralPushPullBrain: Odd number outputs.");
			System.exit (0);
		}

		m_network = network_;
	}

	public List<Float> readOutput (float tpf_)
	{
		List<Float> output = new ArrayList<Float> ();
		List<Neuron> oNeurons = m_network.getOutputNeurons ();

		for (int i = 0; i < oNeurons.size (); i += 2)
		{
			output.add ((float) (oNeurons.get (i).getMembranePotential () -
			                     oNeurons.get (i + 1).getMembranePotential ()));
		}

		return output;
	}

	public void activate (List<Float> input_, float tpf_)
	{
		if (input_.size () != m_network.getInputNeurons ().size ())
		{
			System.out.println ("NeuralPushPullBrain::activate: Size mismatch");
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
		return 2 * outputs_;
	}

	private final Network m_network;
}

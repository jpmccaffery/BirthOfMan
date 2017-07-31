package critter.brain;

import network.Network;


public abstract class NeuralBrainVat implements BrainVat
{
	protected NeuralBrainVat (Class type_)
	{
		m_type = type_;
	}

	public Brain grow (int numInputs_, int numOutputs_)
	{
		int in = 0;
		int out = 0;

		// Switch over brain types
		if (m_type == NeuralStraightBrain.class)
		{
			in = NeuralStraightBrain.nnInputs (numInputs_);
			out = NeuralStraightBrain.nnOutputs (numOutputs_);

			return new NeuralStraightBrain (getNetwork (in, out));
		}
		else if (m_type == NeuralPushPullBrain.class)
		{
			in = NeuralPushPullBrain.nnInputs (numInputs_);
			out = NeuralPushPullBrain.nnOutputs (numOutputs_);

			return new NeuralPushPullBrain (getNetwork (in, out));
		}

		System.err.println ("NeuralBrainVat::grow: Unknown brain type.");
		System.exit (0);

		return null;
	}

	protected abstract Network getNetwork (int numInputs_, int numOutputs_);

	private final Class m_type;
}

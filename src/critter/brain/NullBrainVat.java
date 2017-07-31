package critter.brain;

import network.Network;


public abstract class NullBrainVat
{
	public NullBrainVat ()
	{
	}

	public Brain grow (int numInputs_, int numOutputs_)
	{
		return new NullBrain (numOutputs_);
	}
}

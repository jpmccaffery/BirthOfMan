package critter.brain;


public class NullBrainVat implements BrainVat
{
	public NullBrainVat ()
	{
	}

	public Brain grow (int numInputs_, int numOutputs_)
	{
		return new NullBrain (numOutputs_);
	}
}

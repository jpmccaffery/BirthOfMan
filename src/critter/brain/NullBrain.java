package critter.brain;

import java.util.ArrayList;
import java.util.List;


public class NullBrain implements Brain
{
	public NullBrain (int outputs_)
	{
		m_outputs = outputs_;
	}

	public List<Float> readOutput (float tpf_)
	{
		List<Float> output = new ArrayList<Float> ();

		for (int i = 0; i < m_outputs; i++)
			output.add (0f);

		return output;
	}

	public void activate (List<Float> input_, float tpf_)
	{
	}

	private final int m_outputs;
}

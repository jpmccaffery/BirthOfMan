package critter.brain;

import java.util.List;


public interface Brain
{
	public List<Float> readOutput (float tpf_);
	public void activate (List<Float> input_, float tpf_);

	// These should be static
//	public int nnInputs (int inputs_);
//	public int nnOutputs (int outputs_);
}

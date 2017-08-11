package critter.actuator;

import java.util.List;


public interface Actuator
{
	public void act (List<Float> input_, float tpf_);
	public int size ();

	public void highlight ();
	public void unHighlight ();

	public void updateHighlight ();
}

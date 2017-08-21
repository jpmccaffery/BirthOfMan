package critter.actuator;

import physics.Highlightable;

import java.util.List;


public interface Actuator extends Highlightable
{
	public void act (List<Float> input_, float tpf_);
	public int size ();
}

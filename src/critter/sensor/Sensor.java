package critter.sensor;

import java.util.List;


public interface Sensor
{
	public List<Float> read (float tpf_);
	public int size ();
}

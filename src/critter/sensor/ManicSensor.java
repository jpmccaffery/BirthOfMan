package critter.sensor;

import java.util.ArrayList;
import java.util.List;


public class ManicSensor implements Sensor
{
	public ManicSensor (int size_)
	{
		m_size = size_;
	}

	public List<Float> read (float tpf_)
	{
		List<Float> output = new ArrayList<Float> ();

		for (int i = 0; i < m_size; i++)
			output.add (1f);

		return output;
	}

	public int size ()
	{
		return m_size;
	}

	private int m_size;
}

package critter;

import critter.actuator.Actuator;
import critter.body.Body;
import critter.brain.Brain;
import critter.sensor.Sensor;

import java.util.ArrayList;
import java.util.List;

public class Critter
{
	public Critter (Brain brain_, Body body_, List<Sensor> sensors_,
			List<Actuator> actuators_)
	{
		m_brain = brain_;
		m_body = body_;
		m_sensors = sensors_;
		m_actuators = actuators_;
	}

	public void think (float tpf_)
	{
		List<Float> inputActivity = new ArrayList<Float> ();

		for (Sensor s : m_sensors)
		{
			for (float i : s.read (tpf_))
			{
				inputActivity.add (i);
			}
		}

		m_brain.activate (inputActivity, tpf_);
	}

	public void act (float tpf_)
	{
		List<Float> outputActivity = m_brain.readOutput (tpf_);
		List<Float> localOutput = new ArrayList<Float> ();
		int outputIndex = 0;

		for (Actuator actuator : m_actuators)
		{
			while (localOutput.size () < actuator.size ())
			{
				localOutput.add (outputActivity.get (outputIndex));
				outputIndex++;
			}

			actuator.act (localOutput, tpf_);
			localOutput.clear ();
		}
	}

	public Body body ()
	{
		return m_body;
	}

	private final Brain m_brain;

	private final Body m_body;
	private final List<Sensor> m_sensors;
	private final List<Actuator> m_actuators;
}

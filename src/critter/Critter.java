package critter;

import critter.actuator.Actuator;
import critter.body.Body;
import critter.brain.Brain;
import critter.brain.NullBrain;
import critter.sensor.Sensor;

import java.util.ArrayList;
import java.util.List;

public class Critter
{
	public class ControlSystem
	{
		private ControlSystem ()
		{
		}

		public Actuator first ()
		{
			if (m_actuators.isEmpty ())
				return null;

			return m_actuators.get (0);
		}

		public Actuator next (Actuator a_)
		{
			int index = m_actuators.indexOf (a_);

			if (index == -1 || index == (m_actuators.size () - 1))
				return null;

			return m_actuators.get (index + 1);
		}
	}

	///////////////////////////////////////////////////////////////////////////
	public Critter (Brain brain_, Body body_, List<Sensor> sensors_,
			List<Actuator> actuators_)
	{
		m_brain = brain_;
		m_body = body_;
		m_sensors = sensors_;
		m_actuators = actuators_;

		m_controlSystem = new ControlSystem ();
	}

	public void think (float tpf_)
	{
		if (m_brain instanceof NullBrain)
			return;

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
		if (m_brain instanceof NullBrain)
			return;

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

	public ControlSystem controls ()
	{
		return m_controlSystem;
	}

	private final Brain m_brain;

	private final Body m_body;
	private final List<Sensor> m_sensors;
	private final List<Actuator> m_actuators;

	private final ControlSystem m_controlSystem;
}

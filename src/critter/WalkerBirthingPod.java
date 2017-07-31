package critter;

import critter.actuator.Actuator;
import critter.actuator.JointActuator;
import critter.body.Body;
import critter.brain.BrainVat;
import critter.sensor.JointSensor;
import critter.sensor.ManicSensor;
import critter.sensor.Sensor;

import physics.Joint;

import java.util.List;
import java.util.ArrayList;


public class WalkerBirthingPod extends BirthingPod
{
	public WalkerBirthingPod (BrainVat vat_, Body body_)
	{
		m_brainVat = vat_;
		m_body = body_;

		m_sensors = new ArrayList<Sensor> ();

		for (Joint j : m_body.joints ())
			m_sensors.add (new JointSensor (j));

		m_sensors.add (new ManicSensor (1));

		m_actuators = new ArrayList<Actuator> ();

		for (Joint j : m_body.joints ())
			m_actuators.add (new JointActuator (j));
	}

	public BrainVat brainVat ()
	{
		return m_brainVat;
	}

	public List<Sensor> sensors ()
	{
		return m_sensors;
	}

	public List<Actuator> actuators ()
	{
		return m_actuators;
	}

	public Body body ()
	{
		return m_body;
	}

	private BrainVat m_brainVat;
	private List<Sensor> m_sensors;
	private List<Actuator> m_actuators;
	private Body m_body;
}

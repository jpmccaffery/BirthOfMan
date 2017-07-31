package critter;

import critter.actuator.Actuator;
import critter.body.Body;
import critter.brain.Brain;
import critter.brain.BrainVat;
import critter.sensor.Sensor;

import java.util.List;

public abstract class BirthingPod
{
	public Critter birth ()
	{
		Brain newBrain = brainVat ().grow (inputSize (), outputSize ());

		return new Critter (newBrain, body (), sensors (), actuators ());
	}

	private int inputSize ()
	{
		int total = 0;

		for (Sensor s : sensors ())
			total += s.size ();

		return total;
	}

	private int outputSize ()
	{
		int total = 0;

		for (Actuator a : actuators ())
			total += a.size ();

		return total;
	}


	public abstract BrainVat brainVat ();
	public abstract List<Sensor> sensors ();
	public abstract List<Actuator> actuators ();
	public abstract Body body ();
}

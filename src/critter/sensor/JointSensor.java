package critter.sensor;

import physics.Joint;

import java.util.ArrayList;
import java.util.List;


public class JointSensor implements Sensor
{
	public JointSensor (Joint joint_)
	{
		m_joint = joint_;
	}

	public List<Float> read (float tpf_)
	{
		List<Float> output = new ArrayList<Float> ();

		output.add (m_joint.angularSpeed ().getX ());
		output.add (m_joint.angularSpeed ().getY ());
		output.add (m_joint.angularSpeed ().getZ ());

		return output;
	}

	public int size ()
	{
		return 3;
	}

	private Joint m_joint;
}

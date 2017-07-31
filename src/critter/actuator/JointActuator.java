package critter.actuator;

import physics.Joint;

import com.jme3.math.Vector3f;

import java.util.ArrayList;
import java.util.List;


public class JointActuator implements Actuator
{
	public JointActuator (Joint joint_)
	{
		m_joint = joint_;
	}

	public void act (List<Float> input_, float tpf_)
	{
		if (input_.size () != 3)
		{
			System.err.println ("JointActuator::act: Invalid size.");
			System.exit (0);
		}

		Vector3f action = new Vector3f (input_.get (0), input_.get (1), input_.get (2));

		m_joint.setAngularSpeed (action);
	}

	public int size ()
	{
		return 3;
	}

	private Joint m_joint;
}

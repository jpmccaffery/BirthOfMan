package critter.body;

import physics.Joint;
import physics.Limb;

import com.jme3.math.Vector3f;

import java.lang.Math;

import java.util.ArrayList;
import java.util.List;


public class Snake extends Body
{
	public Snake ()
	{
		m_limbs = new ArrayList<Limb> ();
		m_joints = new ArrayList<Joint> ();

		float length = 0.75f;
		float mass = 1f;
		Limb leg;
		Limb lastLeg;
		Joint hip;

		m_torso = new Limb (new Vector3f (0f, 0f, 0f), new Vector3f (0f, 0f, 1f),
		                    length, mass, 2);
		m_limbs.add (m_torso);

		// Add the back legs
		lastLeg = new Limb (new Vector3f (0f, 0f, 2 * length),
		                    new Vector3f (0f, 0f, 1f), length, mass, 2);
		m_limbs.add (lastLeg);

		hip = new Joint (lastLeg, m_torso);
		m_joints.add (hip);

		leg = new Limb (new Vector3f (0f, 0f, 4 * length),
		                new Vector3f (0f, 0f, 1f), length, mass, 1);
		m_limbs.add (leg);

		hip = new Joint (leg, lastLeg);
		m_joints.add (hip);

		// Add the front legs
		lastLeg = new Limb (new Vector3f (0f, 0f, -2 * length),
		                    new Vector3f (0f, 0f, 1f), length, mass, 2);
		m_limbs.add (lastLeg);

		hip = new Joint (lastLeg, m_torso);
		m_joints.add (hip);

		leg = new Limb (new Vector3f (0f, 0f, -4 * length),
		                new Vector3f (0f, 0f, 1f), length, mass, 1);
		m_limbs.add (leg);

		hip = new Joint (leg, lastLeg);
		m_joints.add (hip);
	}

	@Override
	public List<Limb> limbs ()
	{
		return m_limbs;
	}

	@Override
	public List<Joint> joints ()
	{
		return m_joints;
	}

	@Override
	public Vector3f position ()
	{
		return m_torso.position ();
	}

	private Limb m_torso;

	private List<Limb> m_limbs;
	private List<Joint> m_joints;
}

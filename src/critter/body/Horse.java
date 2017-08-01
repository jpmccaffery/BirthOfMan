package critter.body;

import physics.Joint;
import physics.Limb;

import com.jme3.math.Vector3f;

import java.lang.Math;

import java.util.ArrayList;
import java.util.List;


public class Horse extends Body
{
	public Horse ()
	{
		m_limbs = new ArrayList<Limb> ();
		m_joints = new ArrayList<Joint> ();

		float length = 0.75f;
		float torsoLength = 3 * length;
		float mass = 1f;
		Limb leg;
		Joint hip;

		m_torso = new Limb (new Vector3f (0f, 0f, 0f), new Vector3f (0f, 0f, 1f),
		                    torsoLength, mass, 2);
		m_limbs.add (m_torso);

		float halfLength = torsoLength / 2f;
		float offset = (float) (halfLength / Math.sqrt (2f));

		// Add the back legs
		leg = new Limb (new Vector3f (offset, -offset, -halfLength),
		                new Vector3f (1f, -1f, 0f), length, mass, 1);
		m_limbs.add (leg);

		hip = new Joint (leg, m_torso);
		m_joints.add (hip);

		leg = new Limb (new Vector3f (-offset, -offset, -halfLength),
		                new Vector3f (-1f, -1f, 0f), length, mass, 1);
		m_limbs.add (leg);

		hip = new Joint (leg, m_torso);
		m_joints.add (hip);

		// Add the front legs
		leg = new Limb (new Vector3f (offset, -offset, halfLength),
		                new Vector3f (1f, -1f, 0f), length, mass, 1);
		m_limbs.add (leg);

		hip = new Joint (leg, m_torso);
		m_joints.add (hip);

		leg = new Limb (new Vector3f (-offset, -offset, halfLength),
		                new Vector3f (-1f, -1f, 0f), length, mass, 1);
		m_limbs.add (leg);

		hip = new Joint (leg, m_torso);
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

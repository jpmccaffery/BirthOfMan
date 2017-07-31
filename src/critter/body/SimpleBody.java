package critter.body;

import physics.Joint;
import physics.Limb;

import com.jme3.math.Vector3f;

import java.util.ArrayList;
import java.util.List;


public class SimpleBody extends Body
{
	public SimpleBody ()
	{
		m_limbs = new ArrayList<Limb> ();
		m_joints = new ArrayList<Joint> ();

		float length = 0.75f;
		float mass = 1f;
		Limb arm;
		Joint elbow;

		m_torso = new Limb (new Vector3f (0f, 0f, 0f), new Vector3f (0f, 0f, 1f),
                            length, mass, 2);
		m_limbs.add (m_torso);

		float offset = length / 2f + length / 2f;

		arm = new Limb (new Vector3f (0f, -offset, -offset), new Vector3f (0f, -1f, 0f),
                        length, mass, 1);
		m_limbs.add (arm);

		elbow = new Joint (arm, m_torso);
		m_joints.add (elbow);

		arm = new Limb (new Vector3f (0f, -offset, offset), new Vector3f (0f, -1f, 0f),
                        length, mass, 1);
		m_limbs.add (arm);

		elbow = new Joint (arm, m_torso);
		m_joints.add (elbow);
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

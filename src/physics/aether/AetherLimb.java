package physics.aether;

import physics.AbstractLimb;

import utilities.CoordUtils;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;


public class AetherLimb implements AbstractLimb, AetherObject
{
	public AetherLimb (AbstractLimb limb_)
	{
		m_orientation = limb_.orientation ();
		m_speed = limb_.speed ();
		m_length = limb_.length ();
		m_mass = limb_.mass ();
		m_numCaps = limb_.numCaps ();

		m_position = limb_.position ();
	}

	public AetherLimb (Vector3f position_, Vector3f alignment_, float length_, float mass_,
	                   int numCaps_)
	{
		m_orientation = CoordUtils.rotationFromZAxis (alignment_);
		m_speed = new Vector3f (0f, 0f, 0f);
		m_length = length_;
		m_mass = mass_;
		m_numCaps = numCaps_;

		m_position = position_;
	}

	public AetherLimb (Vector3f position_, Vector3f alignment_, float length_, float mass_)
	{
		this (position_, alignment_, length_, mass_, 0);

		m_speed = new Vector3f (0f, 0f, 0f);
	}

	public AetherLimb (Vector3f position_, float length_, float mass_)
	{
		this (position_, new Vector3f (0f, 0f, 1f), length_, mass_, 0);

		m_speed = new Vector3f (0f, 0f, 0f);
	}

	public AetherLimb (Vector3f position_, float length_)
	{
		this (position_, new Vector3f (0f, 0f, 1f), length_, 1f, 0);

		m_speed = new Vector3f (0f, 0f, 0f);
	}

	public AetherLimb (float length_)
	{
		this (new Vector3f (0f, 0f, 0f), new Vector3f (0f, 0f, 1f), length_, 1f, 0);

		m_speed = new Vector3f (0f, 0f, 0f);
	}

	public AetherLimb ()
	{
		this (new Vector3f (0f, 0f, 0f), new Vector3f (0f, 0f, 1f), 1f, 1f, 0);

		m_speed = new Vector3f (0f, 0f, 0f);
	}

	public Vector3f speed ()
	{
		return m_speed;
	}

	public void setSpeed (Vector3f speed_)
	{
		m_speed = speed_;
	}

	public float length ()
	{
		return m_length;
	}

	public void setLength (float length_)
	{
		m_length = length_;
	}

	public float mass ()
	{
		return m_mass;
	}

	public void setMass (float mass_)
	{
		m_mass = mass_;
	}

	public Quaternion orientation ()
	{
		return m_orientation;
	}

	public void setOrientation (Quaternion orientation_)
	{
		m_orientation = orientation_;
	}

	public Vector3f alignment ()
	{
		return CoordUtils.zAxisFromRotation (m_orientation);
	}

	public void setAlignment (Vector3f alignment_)
	{
		m_orientation = CoordUtils.rotationFromZAxis (alignment_);
	}

	public int numCaps ()
	{
		return m_numCaps;
	}

	public Vector3f position ()
	{
		return m_position;
	}

	public void setPosition (Vector3f position_)
	{
		m_position = position_;
	}

	public void unregisterFromSpace ()
	{
	}

	private Vector3f m_speed;
	private float m_mass;
	private float m_length;
	private int m_numCaps;
	private Quaternion m_orientation;

	private Vector3f m_position;
}

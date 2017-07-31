package physics;

import com.jme3.math.Vector3f;
import com.jme3.math.Quaternion;


public interface AbstractLimb extends PhysicsObject
{
	public Vector3f speed ();
	public void setSpeed (Vector3f speed_);

	public float length ();
	public void setLength (float length_);

	public float mass ();
	public void setMass (float mass_);

	public Quaternion orientation ();
	public void setOrientation (Quaternion orientation_);

	public Vector3f alignment ();
	public void setAlignment (Vector3f alignment_);

	public int numCaps ();

	public void setPosition (Vector3f position_);
}

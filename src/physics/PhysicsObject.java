package physics;

import com.jme3.math.Vector3f;


/// \brief Basic physics object
public interface PhysicsObject
{
	public Vector3f position ();
	public void unregisterFromSpace ();
}

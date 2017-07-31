package physics;

import com.jme3.math.Vector3f;


public interface AbstractJoint extends PhysicsObject
{
	public Vector3f angularSpeed ();
	public void setAngularSpeed (Vector3f speed_);

	public Limb leftJoin ();
	public Limb rightJoin ();
}

package physics.bullet;

import physics.PhysicsObject;


/// \brief Basic Bullet physics object
public interface BulletObject extends PhysicsObject
{
	public void registerWithBullet (DummyBulletSpace space_);
}

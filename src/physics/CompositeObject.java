package physics;

import physics.aether.AetherObject;

import physics.bullet.BulletObject;

import physics.jmonkey.MonkeyObject;


public interface CompositeObject extends AetherObject, BulletObject, MonkeyObject
{
}

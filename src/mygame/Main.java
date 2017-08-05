package mygame;

import evolver.CreaturesMain;

import com.bulletphysics.collision.shapes.SphereShape;

// Test git on windows
public class Main
{
	public static void main (String[] args_)
	{
		SphereShape sphere = new SphereShape (71);
		System.out.println (sphere.getRadius ());

		CreaturesMain app = new CreaturesMain ();

		app.setPauseOnLostFocus (false);
		app.start ();
	}
}

package utilities;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;


public class CoordUtils
{
	public static Quaternion rotationFromZAxis (Vector3f zAxis_)
	{
		float a = zAxis_.getX ();
		float b = zAxis_.getY ();
		float c = zAxis_.getZ ();

		Vector3f xAxis;
		Vector3f yAxis;

		if (a == 0f && b == 0f)
			xAxis = new Vector3f (1f, 0f, 0f);
		else if (a == 0f && c == 0f)
			xAxis = new Vector3f (0f, 0f, 1f);
		else if (b == 0f && c == 0f)
			xAxis = new Vector3f (0f, 1f, 0f);
		else if (a == 0f)
			xAxis = new Vector3f (0f, 1f, -b / c);
		else if (b == 0f)
			xAxis = new Vector3f (1f, 0f, -a / c);
		else if (c == 0f)
			xAxis = new Vector3f (1f, -a / b, 0f);
		else
			xAxis = new Vector3f (1f, 1f, -(a + b) / c);

		yAxis = zAxis_.cross (xAxis);

		xAxis.normalizeLocal ();
		yAxis.normalizeLocal ();
		zAxis_.normalizeLocal ();

		return (new Quaternion ()).fromAxes (xAxis, yAxis, zAxis_);
	}

	public static Vector3f zAxisFromRotation (Quaternion rotation_)
	{
		return rotation_.mult (new Vector3f (0f, 0f, 1f));
	}
}

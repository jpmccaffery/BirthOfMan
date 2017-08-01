package critter;

import critter.body.Snake;
import critter.brain.BrainVat;

import java.util.List;
import java.util.ArrayList;


public class SnakeBirthingPod extends WalkerBirthingPod
{
	public SnakeBirthingPod (BrainVat vat_)
	{
		super (vat_, new Snake ());
	}
}

package critter;

import critter.body.Horse;
import critter.brain.BrainVat;

import java.util.List;
import java.util.ArrayList;


public class HorseBirthingPod extends WalkerBirthingPod
{
	public HorseBirthingPod (BrainVat vat_)
	{
		super (vat_, new Horse ());
	}
}

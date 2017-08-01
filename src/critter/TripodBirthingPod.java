package critter;

import critter.body.Tripod;
import critter.brain.BrainVat;

import java.util.List;
import java.util.ArrayList;


public class TripodBirthingPod extends WalkerBirthingPod
{
	public TripodBirthingPod (BrainVat vat_)
	{
		super (vat_, new Tripod ());
	}
}

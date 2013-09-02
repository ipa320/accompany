package it.unisi.accompany.msgs_and_data;

public class AP_couple {
	protected final double LIKELIHOOD_TRS=1.0;
	
	protected int id;
	protected double likelihood;
	public boolean check;
	
	
	public AP_couple(int idd,double l)
	{
		this.id=idd;
		this.likelihood=l;
		check=false;
	}
	
	public double getLikelihood()
	{
		return likelihood;
	}
	
	public int getId()
	{
		return id;
	}
	
	public boolean compare(AP_couple c)
	{
		if (c.getId()!=id) return false;
		
		if (Math.abs(c.getLikelihood()-likelihood)>LIKELIHOOD_TRS)
			return false;
		
		return true;
	}
}

package it.unisi.accompany.widget.actionliststuffs;

import android.widget.Button;
import it.unisi.accompany.activities.ActionsListView;

public class Action{

	public int ap_id;
	public String name;
	public double likelihood;
	public String environment;
	public String phrase;
	public int act_precondition_id;
	
	public Action(int idd,String n, double like,String p, int precondid,String location)
	{
		
		this.ap_id=idd;
		this.name=n;
		this.likelihood=like;
		if (likelihood<0)
			likelihood=0;
		if (likelihood>1)
			likelihood=1;
		this.phrase=p;
		this.act_precondition_id=precondid;
		this.environment=location;
	}
}

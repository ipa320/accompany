package com.questit.accompany2.data;

public class Action 
{
	public final int SIMPLE=1;
	public final int SEEK=2;
	public final int FATHER=3;
	public final int SON=4;
	
	public String label;
	public String command;
	public int type;
	public String environment;
	public int id;
	public String phrase;
	
	public Action()
	{
		
	}
	
	public Action(String l, String c)
	{
		this.label=l;
		this.command=c;
		this.environment=null;
		this.id=-1;
	}
	
	public Action(String l, String c, String p)
	{
		this.label=l;
		this.command=c;
		this.phrase=p;
		this.environment=null;
		this.id=-1;
	}
	
	public Action (String l, String c, String t, String e,int id_)
	{
		this.label=l;
		this.command=c;
		this.environment=e;
		this.id=id_;
		
		/*if ((t.equals("onoff"))||(t.equals("OnOff")))
		{
			this.type=SIMPLE;
		}
		else
		{
			if (t.equals("seek"))
			{
				this.type=SEEK;
			}
			else
			{
				if (t.equals("father"))
				{
					this.type=FATHER;
				}
				else
				{
					this.type=SON;
				}
			}
		}*/
		
		if ((t.contains("Simple"))||(t.equals("simple")))
		{
			this.type=SIMPLE;
		}
		else
		{
			if ((t.contains("Seek"))||(t.equals("seek")))
			{
				this.type=SEEK;
			}
			else
			{
				if  (((t.contains("Father"))||(t.equals("father")))||
						((t.contains("Parent"))||(t.equals("parent"))))
				{
					this.type=FATHER;
				}
				else
				{
					this.type=SON;
				}
			}
		}
	}
	
	public Action (String l, String c, String t, String e,int id_, String p)
	{
		this.label=l;
		this.command=c;
		this.environment=e;
		this.id=id_;
		this.phrase=p;
		
		/*if ((t.equals("onoff"))||(t.equals("OnOff")))
		{
			this.type=SIMPLE;
		}
		else
		{
			if (t.equals("seek"))
			{
				this.type=SEEK;
			}
			else
			{
				if (t.equals("father"))
				{
					this.type=FATHER;
				}
				else
				{
					this.type=SON;
				}
			}
		}*/
		
		if ((t.contains("Simple"))||(t.equals("simple")))
		{
			this.type=SIMPLE;
		}
		else
		{
			if ((t.contains("Seek"))||(t.equals("seek")))
			{
				this.type=SEEK;
			}
			else
			{
				if  (((t.contains("Father"))||(t.equals("father")))||
						((t.contains("Parent"))||(t.equals("parent"))))
				{
					this.type=FATHER;
				}
				else
				{
					this.type=SON;
				}
			}
		}
	}
}

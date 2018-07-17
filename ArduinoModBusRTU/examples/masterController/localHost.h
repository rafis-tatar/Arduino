class localHost
{	

public:
	byte ID;
	int MinValue;
	bool IsOn = false;
	String CommandState;

	void SetValues(unsigned int values[]);

	localHost(byte id, int minValue);//, Tiny_ModBusRTU_Master *master);
	~localHost();

private:	
	unsigned int regTable[3];
};

inline void localHost::SetValues(unsigned int values[])
{
	regTable[0] = values[0];
	regTable[1] = values[1];
	regTable[2] = values[2];
}

localHost::localHost(byte id, int minValue)// , Tiny_ModBusRTU_Master *master)
{
	ID = id;
	MinValue = minValue;		
}

localHost::~localHost()
{
}




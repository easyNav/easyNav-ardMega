/*
 * myutil.cpp
 *
 * Created: 24/9/2014 6:03:22 PM
 *  Author: adnan
 */ 

void convertToDecimalString(char * buf, int n)
{

	char * str = buf;
	char c;
	int m;
	
	buf[1] = 0;
	buf[2] = 0;
	
	if(n > 99)
	{
		// 3-digit no.
		str += 2;
	}
	else if(n > 9)
	{
		// 2-digit no.
		str += 1;
	}

	do 
	{
		m = n;
		n /= 10;
		c = m - 10 * n;
		*str = (c < 10) ? (c + '0') : (c + 'A' - 10);
		--str;
	} 
	while(n);
	
}


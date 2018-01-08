#if !defined(IVYAPPLICATION_H)
#define IVYAPPLICATION_H


// IvyApplication.h : header file
//


#include "Ivycpp.h"


/////////////////////////////////////////////////////////////////////////////
// IvyApplication command target

class IvyApplication 
{
// Attributes
public:

IvyC::IvyClientPtr appptr;
	
// Operations
public:
	IvyApplication(IvyC::IvyClientPtr ptr );
	virtual ~IvyApplication();

// Overrides
public:
	const char *GetName(void);
	const char *GetHost(void);



};

/////////////////////////////////////////////////////////////////////////////

#endif // !defined(IVYAPPLICATION_H)





// IvyApplication.cpp : implementation file
//

#include "IvyApplication.h"

/////////////////////////////////////////////////////////////////////////////
// IvyApplication

IvyApplication::IvyApplication(IvyC::IvyClientPtr ptr)
{
appptr = ptr;
}

IvyApplication::~IvyApplication()
{

}


const char *IvyApplication::GetName(void)
{
return IvyC::IvyGetApplicationName( appptr );
}


const char *IvyApplication::GetHost(void)
{
return IvyC::IvyGetApplicationHost( appptr );
}






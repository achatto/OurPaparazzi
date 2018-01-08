// Ivy.cpp: implementation of the Ivy class.
//
//////////////////////////////////////////////////////////////////////


#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

// DEBUG
#include <sys/types.h>
#include <unistd.h>
// END DEBUG

#include "Ivycpp.h"
#include "IvyApplication.h"
#include "Ivy/version.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

/*
#                    _
#                   | |
#          ___      | |_     ___   _   _   _ __
#         / __|     | __|   / _ \ | | | | | '__|
#        | (__   _  \ |_   |  __/ | |_| | | |
#         \___| (_)  \__|   \___|  \__,_| |_|
*/
Ivy::Ivy()
{
}


Ivy::Ivy(const char* name, const char * ready, IvyApplicationCallback *callback,
	 bool argaFrc, IvyC::IvyDieCallback _dieCB)
{
  IvyC::IvyInit( name, ready, ApplicationCb, callback, 
 		 _dieCB ? _dieCB :  DieCb, 
 		 callback);
}

/*
#         _____         _
#        |  __ \       | |
#        | |  | |      | |_     ___   _   _   _ __
#        | |  | |      | __|   / _ \ | | | | | '__|
#        | |__| |   _  \ |_   |  __/ | |_| | | |
#        |_____/   (_)  \__|   \___|  \__,_| |_|
*/
Ivy::~Ivy()
{

}

	
/*
#                _                     _
#               | |                   | |
#         ___   | |_     __ _   _ __  | |_
#        / __|  | __|   / _` | | '__| | __|
#        \__ \  \ |_   | (_| | | |    \ |_
#        |___/   \__|   \__,_| |_|     \__|
 */
void Ivy::start(const char *domain)
{
  IvyC::IvyStart( domain );  
}



void  Ivy::ivyMainLoop (void)
{
  IvyC::IvyMainLoop();
}


/*
#                _              _ __
#               | |            | '_ \
#         ___   | |_     ___   | |_) |
#        / __|  | __|   / _ \  | .__/
#        \__ \  \ |_   | (_) | | |
#        |___/   \__|   \___/  |_|
 */
void Ivy::stop()
{
	IvyC::IvyStop( );
}



/*
#         _       _                _    __  __           __ _
#        | |     (_)              | |  |  \/  |         / _` |
#        | |__    _    _ __     __| |  | \  / |  ___   | (_| |
#        | '_ \  | |  | '_ \   / _` |  | |\/| | / __|   \__, |
#        | |_) | | |  | | | | | (_| |  | |  | | \__ \    __/ |
#        |_.__/  |_|  |_| |_|  \__,_|  |_|  |_| |___/   |___/
 */
long Ivy::BindMsg(const char *regexp, IvyMessageCallback *cb)
{
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#endif
  return (long)IvyC::IvyBindMsg( MsgCb , cb, regexp );
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}


long Ivy::BindMsg (IvyMessageCallback *cb, const char *regexp, ... )
{
  char buffer[4096];
  va_list args;
  
  va_start( args, regexp );     /* Initialize variable arguments. */
  vsprintf( buffer, regexp, args );
  va_end( args);
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#endif
  return (long)IvyC::IvyBindMsg( MsgCb , cb, buffer );
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}


/*
#         _    _           _       _                _    __  __           __ _
#        | |  | |         | |     (_)              | |  |  \/  |         / _` |
#        | |  | |  _ __   | |__    _    _ __     __| |  | \  / |  ___   | (_| |
#        | |  | | | '_ \  | '_ \  | |  | '_ \   / _` |  | |\/| | / __|   \__, |
#        | |__| | | | | | | |_) | | |  | | | | | (_| |  | |  | | \__ \    __/ |
#         \____/  |_| |_| |_.__/  |_|  |_| |_|  \__,_|  |_|  |_| |___/   |___/
 */
void Ivy::UnbindMsg(int id)
{
  IvyC::IvyUnbindMsg( (IvyC::MsgRcvPtr) long(id) );
}




/*
#         ____    _                _    _____     _                         _
#        |  _ \  (_)              | |  |  __ \   (_)                       | |
#        | |_) |  _    _ __     __| |  | |  | |   _    _ __    ___    ___  | |_
#        |  _ <  | |  | '_ \   / _` |  | |  | |  | |  | '__|  / _ \  / __| | __|
#        | |_) | | |  | | | | | (_| |  | |__| |  | |  | |    |  __/ | (__  \ |_
#        |____/  |_|  |_| |_|  \__,_|  |_____/   |_|  |_|     \___|  \___|  \__|
#         __  __           __ _
#        |  \/  |         / _` |
#        | \  / |  ___   | (_| |
#        | |\/| | / __|   \__, |
#        | |  | | \__ \    __/ |
#        |_|  |_| |___/   |___/
 */
void Ivy::BindDirectMsg(IvyDirectMessageCallback *callback)
{
	IvyC::IvyBindDirectMsg( MsgDirectCb , callback);
}




/*
#         ______                      _    __  __           __ _
#        /  ____|                    | |  |  \/  |         / _` |
#        | (___     ___   _ __     __| |  | \  / |  ___   | (_| |
#         \___ \   / _ \ | '_ \   / _` |  | |\/| | / __|   \__, |
#        .____) | |  __/ | | | | | (_| |  | |  | | \__ \    __/ |
#        \_____/   \___| |_| |_|  \__,_|  |_|  |_| |___/   |___/
 */
int Ivy::SendMsg(const char *fmt, ... )
{
	static IvyC::IvyBuffer buffer = { NULL, 0, 0}; /* Use static mem to eliminate multiple call 
						    to malloc/free */	
	va_list args;

	va_start( args, fmt );     /* Initialize variable arguments. */
	buffer.offset = 0;
	make_message( &buffer, fmt, args );
	va_end( args);

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#endif
	return IvyC::IvySendMsg (buffer.data); 
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}


/*
#         ______                      _    _____     _                         _
#        /  ____|                    | |  |  __ \   (_)                       | |
#        | (___     ___   _ __     __| |  | |  | |   _    _ __    ___    ___  | |_
#         \___ \   / _ \ | '_ \   / _` |  | |  | |  | |  | '__|  / _ \  / __| | __|
#        .____) | |  __/ | | | | | (_| |  | |__| |  | |  | |    |  __/ | (__  \ |_
#        \_____/   \___| |_| |_|  \__,_|  |_____/   |_|  |_|     \___|  \___|  \__|
#         __  __           __ _
#        |  \/  |         / _` |
#        | \  / |  ___   | (_| |
#        | |\/| | / __|   \__, |
#        | |  | | \__ \    __/ |
#        |_|  |_| |___/   |___/
 */
void Ivy::SendDirectMsg(IvyApplication * app, int id, const char *message)
{
	IvyC::IvySendDirectMsg( app->appptr, id, (char *)message ); 
}



/*
#          _____   _
#         / ____| | |
#        | |      | |    __ _   ___    ___     ___   ___
#        | |      | |   / _` | / __|  / __|   / _ \ / __|
#        | |____  | |  | (_| | \__ \  \__ \  |  __/ \__ \
#         \_____| |_|   \__,_| |___/  |___/   \___| |___/
 */
void Ivy::SetFilter(int argc, const char **argv )
{
	IvyC::IvySetFilter( argc, argv);
}



/*
#          ___    _ __    _ __    _    _                   _      _
#         / _ \  | '_ \  | '_ \  | |  (_)                 | |    (_)
#        | |_| | | |_) | | |_) | | |   _     ___    __ _  | |_    _     ___    _ __
#        |  _  | | .__/  | .__/  | |  | |   / __|  / _` | | __|  | |   / _ \  | '_ \
#        | | | | | |     | |     | |  | |  | (__  | (_| | \ |_   | |  | (_) | | | | |
#        |_| |_| |_|     |_|     |_|  |_|   \___|  \__,_|  \__|  |_|   \___/  |_| |_|
#          _____   _
#         / ____| | |
#        | |      | |__
#        | |      | '_ \
#        | |____  | |_) |
#         \_____| |_.__/
 */
void Ivy::ApplicationCb( IvyC::IvyClientPtr app, void *user_data, IvyC::IvyApplicationEvent event )
{
IvyApplicationCallback *callback = (IvyApplicationCallback *)user_data;
IvyApplication *appObj = new IvyApplication( app );
	switch ( event )
	{
	case IvyC::IvyApplicationConnected:
		callback->OnApplicationConnected( appObj );
	break;
	case IvyC::IvyApplicationDisconnected:
		callback->OnApplicationDisconnected( appObj );
	break;
	case IvyC::IvyApplicationCongestion:
		callback->OnApplicationCongestion( appObj );
	break;
	case IvyC::IvyApplicationDecongestion:
		callback->OnApplicationDecongestion( appObj );
	break;
	case IvyC::IvyApplicationFifoFull:
		callback->OnApplicationFifoFull( appObj );
	break;
	}
delete appObj;
}



/*
#         _____     _            _____   _
#        |  __ \   (_)          / ____| | |
#        | |  | |   _     ___  | |      | |__
#        | |  | |  | |   / _ \ | |      | '_ \
#        | |__| |  | |  |  __/ | |____  | |_) |
#        |_____/   |_|   \___|  \_____| |_.__/
 */
void Ivy::DieCb( IvyC::IvyClientPtr app, void *user_data, int id )
{
}



/*
#         __  __           __ _   _____     _                         _
#        |  \/  |         / _` | |  __ \   (_)                       | |
#        | \  / |  ___   | (_| | | |  | |   _    _ __    ___    ___  | |_
#        | |\/| | / __|   \__, | | |  | |  | |  | '__|  / _ \  / __| | __|
#        | |  | | \__ \    __/ | | |__| |  | |  | |    |  __/ | (__  \ |_
#        |_|  |_| |___/   |___/  |_____/   |_|  |_|     \___|  \___|  \__|
#          _____   _
#         / ____| | |
#        | |      | |__
#        | |      | '_ \
#        | |____  | |_) |
#         \_____| |_.__/
 */
void Ivy::MsgDirectCb( IvyC::IvyClientPtr app, void *user_data, int id, char *msg )
{
IvyDirectMessageCallback *cb = (IvyDirectMessageCallback *)user_data;
IvyApplication *appObj = new IvyApplication( app );
	cb->OnDirectMessage( appObj, id, msg );
delete appObj;
}




/*
#         __  __           __ _    _____   _
#        |  \/  |         / _` |  / ____| | |
#        | \  / |  ___   | (_| | | |      | |__
#        | |\/| | / __|   \__, | | |      | '_ \
#        | |  | | \__ \    __/ | | |____  | |_) |
#        |_|  |_| |___/   |___/   \_____| |_.__/
 */
void Ivy::MsgCb( IvyC::IvyClientPtr app, void *user_data, int argc, char **argv )
{
 
  IvyMessageCallback *cb = (IvyMessageCallback *)user_data;
  IvyApplication *appObj = new IvyApplication( app );
  
  cb->OnMessage( appObj, argc, (const char **)argv );
  delete appObj;

}


void Ivy::BindCallbackCb( IvyC::IvyClientPtr app, void *user_data, int id, const char *regexp,
		     IvyC::IvyBindEvent event) 
{
  IvyBindingCallback *cb = (IvyBindingCallback *)user_data;
  IvyApplication *appObj = new IvyApplication( app );

  switch ( event )
    {
    case IvyC::IvyAddBind :
      cb->OnAddBind ( appObj, id, regexp);
    break;
    case IvyC::IvyRemoveBind :
      cb->OnRemoveBind( appObj, id, regexp );
    break;
    case IvyC::IvyFilterBind :
      cb->OnFilterBind( appObj, id, regexp );
    break;
    case IvyC::IvyChangeBind :
      cb->OnChangeBind( appObj, id, regexp );
    break;
    }
   delete appObj;
}


void Ivy::SetBindCallback(IvyBindingCallback* bind_callback )
{
  IvySetBindCallback (BindCallbackCb,  bind_callback);
}

void Ivy::SetPongCallback(IvyC::IvyPongCallback pong_callback)
{
  IvyC::IvySetPongCallback( pong_callback );
}

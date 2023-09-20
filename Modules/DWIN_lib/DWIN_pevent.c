#include "DWIN_lib.h"

static eDWINEventType eQueuedEvent;
static BOOL     xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xeDWINPortEventInit( void )
{
    xEventInQueue = FALSE;
    return TRUE;
}

BOOL
xDWINPortEventPost( eDWINEventType eEvent )
{
    xEventInQueue = TRUE;
    eQueuedEvent = eEvent;
    return TRUE;
}

BOOL
xDWINPortEventGet( eDWINEventType * eEvent )
{
    BOOL xEventHappened = FALSE;

    if( xEventInQueue )
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = FALSE;
        xEventHappened = TRUE;
    }
    return xEventHappened;
}

BOOL xDWINSetQueue( void ) {
	xEventInQueue = TRUE;
}
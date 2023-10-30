//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynErrorH
#define CDynErrorH
//---------------------------------------------------------------------------

typedef enum 
{
    CDYN_ERROR_FIX,
    CDYN_ERROR_PRIMITIVE_ERROR_TOO_SMALL,
    CDYN_ERROR_PRIMITIVE_NEG_RADIUS,
    CDYN_ERROR_PRIMITIVE_MAX_NUM_VERTICES_EXCEEDED,
    CDYN_ERROR_CLOSED_LOOP,
    CDYN_ERROR_DESTATE_ERROR1,
    CDYN_TIMEHEAP_OVERFLOW
} cDynErrorType;

//---------------------------------------------------------------------------

typedef void (*cbDeError)(cDynErrorType error, void* data);

void cDynErrorSetCallback(cbDeError *f);
void cDynError(cDynErrorType error, void* data = 0);

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
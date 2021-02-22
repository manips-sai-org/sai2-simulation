//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDYNLOGGER_H
#define CDYNLOGGER_H
//---------------------------------------------------------------------------
#include <stdio.h>
//---------------------------------------------------------------------------
#define DFL_ERROR cDynLogger::_DFL_ERROR
#define DFL_WARN  cDynLogger::_DFL_WARN
#define DFL_INFO  cDynLogger::_DFL_INFO
#define DFL_DEBUG cDynLogger::_DFL_DEBUG
//---------------------------------------------------------------------------
#define cDynPrintf cDynLogger::Printf
#define cDynLog cDynLogger::Log
//---------------------------------------------------------------------------
class cDynLogger;
//---------------------------------------------------------------------------

class cDynLoggerOutput
{
    friend class cDynLogger;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    cDynLoggerOutput() 
    {
        next = NULL;
    }

    virtual ~cDynLoggerOutput()
    {
        if (next) delete next;
    }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    virtual void Log(char *msg) = 0;


    //----------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //----------------------------------------------------------------------
protected:

    cDynLoggerOutput *next;
};

//---------------------------------------------------------------------------

class cDynLoggerOutputStd : public cDynLoggerOutput
{
    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    virtual void Log(char *msg);
};

//---------------------------------------------------------------------------

#if defined(WIN32) | defined(WIN64)
class cDynLoggerOutputWinDebug : public cDynLoggerOutput
{
    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    virtual void Log(char *msg);
};
#endif

//---------------------------------------------------------------------------

class cDynLoggerOutputFile : public cDynLoggerOutput
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:
    
    cDynLoggerOutputFile(char* filename);
    ~cDynLoggerOutputFile();
    

    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    virtual void Log(char *msg);


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    FILE *logfile;
};

//---------------------------------------------------------------------------

class cDynLogger
{
public:
    enum
    {
        _DFL_ERROR = 1,
        _DFL_WARN,
        _DFL_INFO,
        _DFL_DEBUG
    };

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:
    cDynLogger();
    virtual ~cDynLogger();


    //----------------------------------------------------------------------
    // PUBLIC STATIC MEMBERS:
    //----------------------------------------------------------------------
public:

    static void Initialize();
    static void Shutdown();

    static void Format(char *buf, int type, char* category, char *msg);
    static void Log(int type, char* category, char* format, ...);
    static void Printf(char* format, ...);

    static void AddOutput(cDynLoggerOutput *out);


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    static cDynLogger* logger;

    cDynLoggerOutput *output;
};

//---------------------------------------------------------------------------
#endif // CDYNLOGGER_H
//---------------------------------------------------------------------------
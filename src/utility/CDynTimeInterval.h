//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynTimeIntervalH
#define CDynTimeIntervalH
//---------------------------------------------------------------------------

#if defined(WIN32) | defined(WIN64)
#include <windows.h>
#else
#include <sys/time.h>
//const double M_SPEED=1e6;
const double M_SPEED=1682.1335844e6;
extern __inline__ unsigned long long int rdtsc()
   {
     unsigned long long int x;
     __asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
     return x;
   }
#endif

//---------------------------------------------------------------------------

class cDynTimeInterval
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

#if defined(WIN32) | defined(WIN64)
    cDynTimeInterval() { if (!Init()) exit(1); }
#endif


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
    void start() { 
#if defined(WIN32) | defined(WIN64)
    ::QueryPerformanceCounter( &liStart_ ); 
#else
    start_time_=Get(); 
#endif
  }

    void end() { 
#if defined(WIN32) | defined(WIN64)
    ::QueryPerformanceCounter( &liFinal_ ); 
#else
    end_time_=Get();
#endif
  }

    double time() {
#if defined(WIN32) | defined(WIN64)
    return GetElapsedTime();
#else
    return((double)(end_time_ - start_time_)/M_SPEED);
#endif
  }
  

    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:
    
#if defined(WIN32) | defined(WIN64)
    //! Machine counter frequency.
    static double frequency_;

    LARGE_INTEGER liStart_;
    LARGE_INTEGER liFinal_;

    //! If the timer is available, return true.
    static bool Init();

    //! Convert LARGE_INTEGER to double.
    static double ConvertLI2D(const LARGE_INTEGER &li);

    // Get the elapsed time. 
    double GetElapsedTime();

#else
    long long start_time_;
    long long end_time_;
    inline long long Get() const;
#endif
};

//---------------------------------------------------------------------------

#if !defined(WIN32) & !defined(WIN64)
inline long long cDynTimeInterval::Get() const
{
#if 1
     return(rdtsc());
#else
    struct timeval tv;
    gettimeofday(&tv, 0x0);
    return(((long long)(tv.tv_sec)) * (long long)(1e6) + (long long)(tv.tv_usec));
#endif
}
#endif

//---------------------------------------------------------------------------
#endif // CDynTimeIntervalH
//---------------------------------------------------------------------------

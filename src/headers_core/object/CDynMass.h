//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynMassH
#define CDynMassH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
#include "matrix/CDynVector3.h"
#include "matrix/CDynMatrix3.h"
#include "matrix/CDynFrameStack.h"
//---------------------------------------------------------------------------
inline double cDynDensity(double v) { return(-v); }
inline bool cDynIsDensity(double v) { return(v < 0.0f); }
//---------------------------------------------------------------------------
// DENSITIES [kg/m^3] FOR COMMON MATERIALS:
//---------------------------------------------------------------------------
const double CDYN_DENSITY_AIR		    =cDynDensity(1.2062f);
const double CDYN_DENSITY_ALUMINUM	    =cDynDensity(2690.0f);
const double CDYN_DENSITY_CONCRETE	    =cDynDensity(2400.0f);
const double CDYN_DENSITY_COPPER		=cDynDensity(8910.0f);
const double CDYN_DENSITY_EARTH_WET	    =cDynDensity(1760.0f);
const double CDYN_DENSITY_EARTH_DRY	    =cDynDensity(1280.0f);
const double CDYN_DENSITY_GLASS		    =cDynDensity(2590.0f);
const double CDYN_DENSITY_GOLD		    =cDynDensity(19300.0f);
const double CDYN_DENSITY_ICE		    =cDynDensity(900.0f);
const double CDYN_DENSITY_IRON		    =cDynDensity(7210.0f);
const double CDYN_DENSITY_LEAD		    =cDynDensity(11370.0f);
const double CDYN_DENSITY_MERCURY	    =cDynDensity(13570.0f);
const double CDYN_DENSITY_OIL		    =cDynDensity(900.0f);
const double CDYN_DENSITY_STEEL		    =cDynDensity(7830.0f);
const double CDYN_DENSITY_TITANIUM	    =cDynDensity(3080.0f);
const double CDYN_DENSITY_WATER		    =cDynDensity(1000.0f);
const double CDYN_DENSITY_WATER_SALT	=cDynDensity(1030.0f);
const double CDYN_DENSITY_WOOD_SOFTPINE	=cDynDensity(480.0f);
const double CDYN_DENSITY_WOOD_HARDOAK	=cDynDensity(800.0f);
//---------------------------------------------------------------------------

class cDynMass
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynMass.    
    cDynMass();
    cDynMass(cDynMass& mass);

    //! Destructor of cDynMass.
    ~cDynMass();


    //----------------------------------------------------------------------
    // PUBLIC OPERATORS:
    //----------------------------------------------------------------------
public:

    cDynMass& operator=(cDynMass& mass);


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (MASS PROPERTIES)
    //----------------------------------------------------------------------
public:

    //! Return inertia of object as seen from objects local frame.
    const cDynMatrix3& inertia() const { return(p->inertia_); }

    //! Return center of mass of object as seen from objects local frame.
    const cDynVector3& center() const { return(p->center_); }

    //! Return current mass parameters.
    double mass() const { return(p->m_); }

    //! Add a point mass of mass \a m at a point at the origin of the reference frame.
    void mass(const double m);

     //! Add an inertia tensor \a inertia specified by a 3x3 matrix at the current frame of reference.
    void inertia(const cDynMatrix3& inertia);
    
    //! Add an inertia tensor specified by its diagonal \a diag = (\a Ixx, \a Iyy, \a Izz).
    void inertia(const cDynVector3& diag); 
    
    //! Add an inertia tensor specified by its diagonal (\a Ixx, \a Iyy, \a Izz).
    void inertia(const double Ixx,const double Iyy,const double Izz);
    
    //! Add an inertia tensor specified by its diagonal \a diag (an array of floats).
    void inertia(const double diag[3]);
        

    //----------------------------------------------------------------------
    // PUBLIC METHODS: (MASS SETTINGS)
    //----------------------------------------------------------------------

    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous cylinder with center
        of mass at the current frame.
        
        \param mp mass (density if cDynDensity(double)) of cylinder.
        \param h  total height of cylinder in z-axis.
        \param r  radius of the cylinder.
   */
   //----------------------------------------------------------------------
    void cylinder(const double mp,const  double h,const  double r);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous cone with center
        of mass at the current frame. The center of mass of a cone is
        located \a h/4 from base in \a z
        
        \param mp mass (density if cDynDensity(double)) of cone.
        \param h  total height of cone in z-axis.
        \param r  radius of the base of the cone.
    */
    //----------------------------------------------------------------------
    void cone(const double mp,const  double h,const  double r);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous four sided pyramid with
        center of mass at the current frame. The center of mass of a pyramid
        is located \a h/4 from base in \a z
        
        \param mp mass (density if cDynDensity(double)) of pyramid.
        \param a  width of base of pyramid along the x-axis.
        \param b  length of base of pyramid along the y-axis.
        \param h  total height of pyramid h in z-axis.
    */
    //----------------------------------------------------------------------
    void pyramid(const double mp,const  double a,const  double b,const  double h);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous block with center of
        mass at the current frame.
        
        \param mp mass (density if cDynDensity(double)) of block.
        \param a  width of base of block along the x-axis.
        \param b  length of base of block along the y-axis.
        \param c  height of base of block along the z-axis.
    */
    //----------------------------------------------------------------------
    void block(const double mp,const  double a,const  double b,const  double c);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous sphere with center of
        mass at the current frame.
        
        \param mp mass (density if cDynDensity(double)) of sphere.
        \param r  radius of sphere.
    */
    //----------------------------------------------------------------------
    void sphere(const double mp,const  double r);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous half sphere (hemisphere) with
        center of mass at the current frame. The center of mass of a hemisphere
        is located \a (3/8)*r from base in \a z
        
        \param mp mass (density if cDynDensity(double)) of hemisphere.
        \param r  radius of hemisphere.
    */
    //----------------------------------------------------------------------
    void hemisphere(const double mp,const  double r);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous ellipsoid with center of
        mass at the current frame.
        
        \param mp mass (density if cDynDensity(double)) of ellipsoid.
        \param a  length of major axis along the x-axis, one half total width
        \param b  length of major axis along the y-axis, one half total length
        \param c  length of major axis along the z-axis, one half total height
    */
    //----------------------------------------------------------------------
    void ellipsoid(const double mp,const  double a,const  double b,const  double c);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous one dimensional rod having
        total length \a l with center of mass at the current frame.
        The center of mass of a rod is located half way along its length.
    
        \param mp mass (linear density if cDynDensity(double)).
        \param l  total length of the rod along the z-axis.
    */
    //----------------------------------------------------------------------
    void rod(const double mp,const  double l);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous two dimensional flat disk
        with center of mass at the current frame. The disk has zero height in
        the z-axis.
        
        \param mp mass (surface density if cDynDensity(double)).
        \param r  radius of disk.
    */
    //----------------------------------------------------------------------
    void disk(const double mp,const double r);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous two dimensional flat 
        square plate with center of mass at the current frame. The plate 
        has zero height in the z-axis.
        
        \param mp mass (surface density if cDynDensity(double)).
        \param a  total width of the in the x-axis.
        \param b  total width of the in the y-axis.
    */
    //----------------------------------------------------------------------
    void plate(const double mp,const  double a,const  double b);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous infinitely
        thin cylinder with no caps, with center of mass at the current frame.
    
        \param mp mass (surface density if cDynDensity(double)).
        \param h  total height of cylinder shell in z-axis.
        \param r  radius of the cylinder shell.
    */
    //----------------------------------------------------------------------
    void cylinderShell(const double mp,const  double h,const  double r);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous infinitely
        thin cone with no base. The center of mass located at the current 
        frame is \a (1/3)*h from base in \a z.
        
        \param mp mass (surface density if cDynDensity(double)).
        \param h  total height of cone shell h in z-axis.
        \param r  radius of the base of the cone shell.
    */
    //----------------------------------------------------------------------
    void coneShell(const double mp,const  double h,const  double r);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous infinitely
        thin sphere, with center of mass at the current frame.
        
        \param mp mass (surface density if cDynDensity(double)).
        \param r  radius of the sphere.
    */
    //----------------------------------------------------------------------
    void sphereShell(const double mp,const  double r);


    //----------------------------------------------------------------------
    /*!
        Specify the mass parameters to a homogeneous infinitely
        thin half sphere (hemisphere) with no base. The center of mass
        located at the current frame is r/2 from the base of the hemisphere 
        in the z-axis.
        
        \param mp mass (surface density if cDynDensity(double)).
        \param r  radius of the hemisphere.
    */
    //----------------------------------------------------------------------
    void hemisphereShell(const double mp,const  double r);


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS: (RESET METHODS)
    //----------------------------------------------------------------------
public:

    //! Reset all the mass properties of the body making mass, and inertia zero.
    void zero();

    //! Same as zero().
    void reset() { zero(); } 

    //! Scale homogeneously the mass prosperities of a given object until total mass equals \a m.
    void scale(const double m);


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS: (SPECIAL METHODS)
    //----------------------------------------------------------------------
public:

    //--------------------------------------------------------------------------
    /*!
        Set frame of reference to be used when specifying a objects mass/inertial 
        properties. \a NULL indicates the identity matrix. Normally set by 
        cDynObject.

        \param fs new frame of reference.
    */
    //--------------------------------------------------------------------------
    void frame(cDynFrameStack *fs) { fs_=fs; }


    //--------------------------------------------------------------------------
    /*!
        Return the current active frame of reference
    
        \returns frame of reference as defined by frame(cDynFrameStack*)
    */
    //--------------------------------------------------------------------------
    cDynFrameStack* frame() { return(fs_); }


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    cDynFrameStack *fs_;
    
    typedef struct 
    {
        double m_;
        cDynVector3 center_;
        cDynMatrix3 inertia_;
        int rc_;
    } Mass;
    
    Mass* p;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
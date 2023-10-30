//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynABNodeH
#define CDynABNodeH
//---------------------------------------------------------------------------
#include "matrix/CDynMatrix3.h"
#include "matrix/CDynMatrix6.h"
#include "matrix/CDynVector6.h"
#include "dynamics/CDynABJoint.h"
//---------------------------------------------------------------------------

class cDynABNode
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual ~cDynABNode() { return; };

    virtual int flag() const = 0;
    virtual void flag(int v) = 0;

//	virtual const cDynMatrix3* Ic() const = 0;
//	virtual const cDynMatrix6* I() const = 0;

    virtual cDynVector6* Pa() = 0;

    virtual cDynVector6* V() { return &_V; }
    virtual cDynVector6* A() { return &_A; }


    //-----------------------------------------------------------------------
    /*!
        Ia = I
    */
    //-----------------------------------------------------------------------
    virtual void abInertiaInit(cDynMatrix6& Ia) = 0;


    //-----------------------------------------------------------------------
    /*!
        Ya = -Xc Yc
           = -[R , 0; dxR , R] [ y, 0]
           = [-Ry ; -dxRy]
    */
    //-----------------------------------------------------------------------
    virtual void impulseInit(const cDynVector3& point, const cDynVector3& impulse) = 0;
    virtual void abImpulseInit() = 0;

    virtual void biasForceConfigInit() = 0;
    virtual void abBiasForceConfigInit() = 0;


    //-----------------------------------------------------------------------
    /*!
        Ii = Xc Ic Xtc
           = [ RMRt, -RMRt rx; rx RMRt, RIRt - rx RMRt rx]
           = [ M, -Mrx; rxM, RIRt - m rx rx]
    */
    //-----------------------------------------------------------------------
    virtual void inertia(const double mass, const cDynVector3& centerOfMass, const cDynMatrix3& inertiaTensor) = 0;


    //-----------------------------------------------------------------------
    /*!
        Pi = Xc (Wc X Ic Vc) - Ii (Wi X Vi)  
           = Xc (Wc X [mi vc; Ic wc]) -Ii (Wi X Vi)
    */
    //-----------------------------------------------------------------------
    virtual void biasForce(cDynVector6& P, const cDynVector6& V, const cDynVector3& WxV) = 0;


    //-----------------------------------------------------------------------
    /*!
        Dinv = inv(St Ia S)
        SbarT = Ia S Dinv
        hLi = hXi [ 1 - Si Sbari ]^T = hXi [1 - Sbari^T Si^T] = X - X SbarT St
        Lt = [1 - S Sbar] Xt
        Iah = Ih + sum [ Li Iai Lti ]
    */
    //-----------------------------------------------------------------------
    virtual void _abInertia(cDynMatrix6& Iah, const cDynMatrix6& L, const cDynMatrix6& Ia) = 0;


    //-----------------------------------------------------------------------
    /*!
        Pah = Ph - Fexth + sum [ Li (Iai Ci + Pai) + X SbarTi taui ]
    */
    //-----------------------------------------------------------------------
    virtual void _abBiasForce(cDynVector6& Pah, const cDynMatrix6& L, const cDynMatrix6& Ia, const cDynVector6& C, const cDynVector6& Pa) = 0;

    virtual void netForce(cDynVector6& F, const cDynVector6& A, const cDynVector6& P) = 0;
    // Pa -= Fext
    virtual void externalForce(cDynVector6& Pa, const cDynVector6& Fext) = 0;

    virtual void updateLocalX(const cDynFrame& homeFrame) = 0;
    virtual void abImpulse(cDynVector6& Yah, int propagate) = 0;


    //-----------------------------------------------------------------------
    /*!
        0Jn = Jn = iXn^T Si = 0Xi^(-T) Si
        where 0Xi^(-T) = [ R rxR; 0 R ]
        since 0Rn = identity
        and  iXn^T = [(inv 0Xi) 0Xn]^T = (0Xn)^T * (0Xi)^(-T) = 0Xi ^(-T)
        since  0Xn = identity matrix    <--  {0} = {e}
    */
    //-----------------------------------------------------------------------
    virtual void globalJacobian(const cDynFrame& globalFrame) = 0;


    //-----------------------------------------------------------------------
    /*!
        Gi = Xc Gc
        Gc = [ mi Rtci gi ; 0]
        gi = Rt gh
    */
    //-----------------------------------------------------------------------
    virtual void gravityForce(cDynVector6& Fext, cDynVector3& g, const cDynVector3& gh) = 0;


    //-----------------------------------------------------------------------
    /*!
        Vi = hXi^T Vh + Si dqi;
        xform = [R 0; dxR R]
        xformT = [ Rt -Rtdx; 0 Rt ]
        Ci = Wi X Vi - Xt (Wh X Vh) + Vi X Si dqi
        V X = [v0 ; v1] X = [ v1x , v0x ; 0 , v1x]
        WxV = [ 0 ; v1 ] x [ v0 ; v1 ]
            = [ v1x , 0 ; 0 , v1x ] [ v0 ; v1 ] = [v1 x v0 ;v1 x v1] = [ v1 x v0 ; 0 ]
            = [ wxv ; 0 ]
        Xt * WxV = [ Rt -Rtdx; 0 Rt ] [ wxv ; 0 ] = [ Rt WxV ; 0 ]
    */
    //-----------------------------------------------------------------------
    virtual void velocity(cDynVector6& V, cDynVector3& WxV, const cDynVector6& Vh, const cDynVector3& WhxVh) = 0;


    //-----------------------------------------------------------------------
    /*!
        Vi = hXi^T Vh + Si dqi;
        E = 0.5 Vt I V
    */
    //-----------------------------------------------------------------------
    virtual double kineticEnergy(cDynVector6& V, const cDynVector6& Vh) = 0;


    //-----------------------------------------------------------------------
    /*!
        E = mgh
    */
    //-----------------------------------------------------------------------
    virtual double potentialEnergy(cDynVector3& g, const cDynVector3& gh, const cDynFrame& globalFrame, const double mass, const cDynVector3& centerOfMass) = 0;


    //-----------------------------------------------------------------------
    /*!
        ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah + Ci)
        Ai = (hXi^T Ah + Ci) + Si ddqi;
    */
    //-----------------------------------------------------------------------
    virtual void acceleration(cDynVector6& A, const cDynVector6& Ah) = 0;
    virtual void accelerationOnly(cDynVector6& A, const cDynVector6& Ah) = 0;


    //-----------------------------------------------------------------------
    /*!
        ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah)
        Ai = (hXi^T Ah) + Si ddqi;
        d dQ = Dinv*(- St*Pa) - Sbar*(X dVh)
        dVi = (hXi^T dVh) + Si d dqi;
    */
    //-----------------------------------------------------------------------
    virtual void velocityDelta(cDynVector6& dV, const cDynVector6& dVh) = 0;

    virtual void force(cDynVector6& Fh, int propagate) = 0;


    //-----------------------------------------------------------------------
    /*!
        C = Ph = 0
        Pah = - Fexth + sum [ Li (Pai) + X SbarTi taui ]
    */
    //-----------------------------------------------------------------------
    virtual void abBiasForceConfig(cDynVector6& Pah, int propagate) = 0;
    virtual void abInertiaDepend(cDynMatrix6& Iah, cDynVector6& Pah, cDynMatrix6& Ia, int propagate) = 0;
    virtual void abInertiaDependConfig(cDynMatrix6& Iah, cDynMatrix6& Ia, int propagate) = 0;

    virtual cDynABJoint* abJoint(int i = 0) = 0;
    virtual void abJoint(cDynABJoint* joint, int i = 0) = 0;

    virtual void noj(int n) = 0;
    virtual int noj() const = 0;


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynVector6 _V;
    cDynVector6 _A;
};

//---------------------------------------------------------------------------

class cDynABNodeRoot : public cDynABNode
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual ~cDynABNodeRoot() { return; };

    virtual int flag() const { return 0; }
    virtual void flag(int v) {}

//	virtual const cDynMatrix3* Ic() const { return NULL; }
//	virtual const cDynMatrix6* I() const { return NULL;}

    virtual cDynVector6* Pa() { return NULL; }

    virtual void abInertiaInit(cDynMatrix6& Ia) {}
    virtual void impulseInit(const cDynVector3& point, const cDynVector3& impulse) {}
    virtual void abImpulseInit() {}

    virtual void biasForceConfigInit() {}
    virtual void abBiasForceConfigInit() {}
    virtual void inertia(const double mass, const cDynVector3& centerOfMass, const cDynMatrix3& inertiaTensor) {}
    virtual void biasForce(cDynVector6& P, const cDynVector6& V, const cDynVector3& WxV) {}
    virtual void _abInertia(cDynMatrix6& Iah, const cDynMatrix6& L, const cDynMatrix6& Ia) {}
    virtual void _abBiasForce(cDynVector6& Pah, const cDynMatrix6& L, const cDynMatrix6& Ia, const cDynVector6& C, const cDynVector6& Pa) {}
    virtual void netForce(cDynVector6& F, const cDynVector6& A, const cDynVector6& P) {}
    virtual void externalForce(cDynVector6& Pa, const cDynVector6& Fext) {}

    virtual void updateLocalX(const cDynFrame& homeFrame) {}
    virtual void abImpulse(cDynVector6& Yah, int propagate) {}
    virtual void globalJacobian(const cDynFrame& globalFrame) {}

    virtual void gravityForce(cDynVector6& Fext, cDynVector3& g, const cDynVector3& gh) { g = gh; }
    virtual void velocity(cDynVector6& V, cDynVector3& WxV, const cDynVector6& Vh, const cDynVector3& WhxVh) { V.zero(); WxV.zero(); }
    virtual double kineticEnergy(cDynVector6& V, const cDynVector6& Vh) { V.zero(); return 0; }
    virtual double potentialEnergy(cDynVector3& g, const cDynVector3& gh, const cDynFrame& globalFrame, const double mass, const cDynVector3& centerOfMass) { g = gh; return 0; }
    virtual void acceleration(cDynVector6& A, const cDynVector6& Ah) { A.zero(); }
    virtual void accelerationOnly(cDynVector6& A, const cDynVector6& Ah) { A.zero(); }
    virtual void velocityDelta(cDynVector6& dV, const cDynVector6& dVh) { dV.zero(); }

    virtual void force(cDynVector6& Fh, int propagate) {}
    virtual void abBiasForceConfig(cDynVector6& Pah, int propagate) {}
    virtual void abInertiaDepend(cDynMatrix6& Iah, cDynVector6& Pah, cDynMatrix6& Ia, int propagate) {}
    virtual void abInertiaDependConfig(cDynMatrix6& Iah, cDynMatrix6& Ia, int propagate) {}

    virtual cDynABJoint* abJoint(int i = 0) { return NULL; }
    virtual void abJoint(cDynABJoint* joint, int i = 0) {}
    virtual void noj(int n) {}
    virtual int noj() const { return 0; }
};

//---------------------------------------------------------------------------

class cDynABNodeNOJ : public cDynABNode
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual ~cDynABNodeNOJ() { return; };

    virtual int flag() const { return _flag; }
    virtual void flag(int v) { _flag = v; }

    virtual const cDynMatrix3* Ic() const { return &_Ic; }
    virtual const cDynMatrix6* I() const { return &_I;}

    virtual cDynVector6* Pa() { return &(abJoint(noj() - 1)->Pa()); }

    virtual void abInertiaInit(cDynMatrix6& Ia);
    virtual void impulseInit(const cDynVector3& point, const cDynVector3& impulse);

    virtual void biasForceConfigInit();
    virtual void abBiasForceConfigInit();
    virtual void inertia(const double mass, const cDynVector3& centerOfMass, const cDynMatrix3& inertiaTensor);
    virtual void biasForce(cDynVector6& P, const cDynVector6& V, const cDynVector3& WxV);
    virtual void _abInertia(cDynMatrix6& Iah, const cDynMatrix6& L, const cDynMatrix6& Ia);
    virtual void _abBiasForce(cDynVector6& Pah, const cDynMatrix6& L, const cDynMatrix6& Ia, const cDynVector6& C, const cDynVector6& Pa);
    virtual void netForce(cDynVector6& F, const cDynVector6& A, const cDynVector6& P);
    virtual void externalForce(cDynVector6& Pa, const cDynVector6& Fext);
    virtual double kineticEnergy(cDynVector6& V, const cDynVector6& Vh);
    virtual double potentialEnergy(cDynVector3& g, const cDynVector3& gh, const cDynFrame& globalFrame, const double mass, const cDynVector3& centerOfMass);

    virtual cDynABJoint* abJoint(int i = 0) = 0;
    virtual void abJoint(cDynABJoint* joint, int i = 0) = 0;
    virtual void noj(int n) = 0;
    virtual int noj() const = 0;


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    int _flag;
    cDynMatrix3 _Ic;
    cDynMatrix6 _I;
};

//---------------------------------------------------------------------------

class cDynABNodeNOJ1 : public cDynABNodeNOJ
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual void abImpulseInit();
    virtual void updateLocalX(const cDynFrame& homeFrame);
    virtual void abImpulse(cDynVector6& Yah, int propagate);
    virtual void globalJacobian(const cDynFrame& globalFrame);

    virtual void gravityForce(cDynVector6& Fext, cDynVector3& g, const cDynVector3& gh);
    virtual void velocity(cDynVector6& V, cDynVector3& WxV, const cDynVector6& Vh, const cDynVector3& WhxVh);
    virtual void acceleration(cDynVector6& A, const cDynVector6& Ah);
    virtual void accelerationOnly(cDynVector6& A, const cDynVector6& Ah);
    virtual void velocityDelta(cDynVector6& dV, const cDynVector6& dVh);

    virtual void force(cDynVector6& Fh, int propagate);
    virtual void abBiasForceConfig(cDynVector6& Pah, int propagate);
    virtual void abInertiaDepend(cDynMatrix6& Iah, cDynVector6& Pah, cDynMatrix6& Ia, int propagate);
    virtual void abInertiaDependConfig(cDynMatrix6& Iah, cDynMatrix6& Ia, int propagate);

    virtual cDynABJoint* abJoint(int i = 0) { return _joint; }
    virtual void abJoint(cDynABJoint* joint, int i = 0) { _joint = joint; }

    virtual void noj(int n) {}
    virtual int noj() const { return 1; }

    virtual ~cDynABNodeNOJ1() { delete _joint; }


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynABJoint* _joint;
};

//---------------------------------------------------------------------------

class cDynABNodeNOJn : public cDynABNodeNOJ
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual void abImpulseInit();
    virtual void updateLocalX(const cDynFrame& homeFrame);
    virtual void abImpulse(cDynVector6& Yah, int propagate);
    virtual void globalJacobian(const cDynFrame& globalFrame);

    virtual void gravityForce(cDynVector6& Fext, cDynVector3& g, const cDynVector3& gh);
    virtual void velocity(cDynVector6& V, cDynVector3& WxV, const cDynVector6& Vh, const cDynVector3& WhxVh);
    virtual void acceleration(cDynVector6& A, const cDynVector6& Ah);
    virtual void accelerationOnly(cDynVector6& A, const cDynVector6& Ah);
    virtual void velocityDelta(cDynVector6& dV, const cDynVector6& dVh);

    virtual void force(cDynVector6& Fh, int propagate);
    virtual void abBiasForceConfig(cDynVector6& Pah, int propagate);
    virtual void abInertiaDepend(cDynMatrix6& Iah, cDynVector6& Pah, cDynMatrix6& Ia, int propagate);
    virtual void abInertiaDependConfig(cDynMatrix6& Iah, cDynMatrix6& Ia, int propagate);

    virtual cDynABJoint* abJoint(int i = 0) { return _joint[i]; }
    virtual void abJoint(cDynABJoint* joint, int i = 0) { _joint[i] = joint; }

    virtual void noj(int n) { _noj = n; _joint = new cDynABJoint*[_noj]; }
    virtual int noj() const { return _noj; }

    virtual ~cDynABNodeNOJn() { delete[] _joint; }


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:
    
    int _noj;
    cDynABJoint** _joint;
};

//---------------------------------------------------------------------------
#endif // CDynABNodeH
//---------------------------------------------------------------------------

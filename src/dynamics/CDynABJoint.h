//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynABJointH
#define CDynABJointH
//---------------------------------------------------------------------------
#include "matrix/CDynMatrix6.h"
#include "matrix/CDynVector6.h"
#include "matrix/CDynVector3.h"
#include "matrix/CDynMatrix3.h"
#include "matrix/CDynQuaternion.h"
//---------------------------------------------------------------------------

class cDynABJoint
{
    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
public:

    virtual ~cDynABJoint() { return; }

    virtual cDynVector6& C() { return _C; }
    virtual cDynVector6& Pa() { return _Pa; }
    virtual cDynTransform& localX() { return _localX; }
    virtual cDynMatrix6& L() { return _L; }

    //-----------------------------------------------------------------------
    /*!
        Vi = hXi^T Vh + Si dqi;
        xform = [R 0; dxR R]
        xformT = [ Rt -Rtdx; 0 Rt ]
    */
    //-----------------------------------------------------------------------
    virtual void update_localX(const cDynTransform& home) = 0;


    //-----------------------------------------------------------------------
    /*!
        Ci = Wi X Vi - Xt (Wh X Vh) + Vi X Si dqi
        V X = [v0 ; v1] X = [ v1x , v0x ; 0 , v1x]
        WxV = [ 0 ; v1 ] x [ v0 ; v1 ]
            = [ v1x , 0 ; 0 , v1x ] [ v0 ; v1 ] = [v1 x v0 ;v1 x v1] = [ v1 x v0 ; 0 ]
            = [ wxv ; 0 ]
        Xt * WxV = [ Rt -Rtdx; 0 Rt ] [ wxv ; 0 ] = [ Rt WxV ; 0 ]
    */
    //-----------------------------------------------------------------------
    virtual void plusEq_SdQ(cDynVector6& V) = 0;

    virtual void plusEq_V_X_SdQ(cDynVector6& C, const cDynVector6& V) = 0;


    //-----------------------------------------------------------------------
    /*!
        Dinv = inv(St Ia S)
        SbarT = Ia S Dinv
        hLi = hXi [ 1 - Si Sbari ]^T = hXi [1 - Sbari^T Si^T] = X - X SbarT St
        Lt = [1 - S Sbar] Xt
    */
    //-----------------------------------------------------------------------
    virtual void compute_Dinv_and_SbarT(const cDynMatrix6& Ia) = 0;

    virtual void minusEq_X_SbarT_St(cDynMatrix6& L, const cDynTransform& localX) = 0;

    //---------------------------------------------------------------------------
    /*!
        Pah = Ph - Fexth + sum [ Li (Iai Ci + Pai) + X SbarTi taui ]
    */
    //---------------------------------------------------------------------------
    virtual void plusEq_X_SbarT_Tau(cDynVector6& Pah, const cDynTransform& localX) = 0;


    //---------------------------------------------------------------------------
    /*!
        ddQ = Dinv*(tau - St*Pa) - Sbar*(X Ah + Ci)
        Ai = (hXi^T Ah + Ci) + Si ddqi;
    */
    //---------------------------------------------------------------------------
    virtual void compute_ddQ(const cDynVector6& Pa, const cDynVector6& XAh_C) = 0;
    

    //---------------------------------------------------------------------------
    /*!
        d dQ = Dinv*(- St*Ya) - Sbar*(X dVh)
        dVi = (hXi^T dVh) + Si d dqi;
    */
    //---------------------------------------------------------------------------
    virtual void compute_ddQ(const cDynVector6& XAh_C) = 0;
    
    virtual void compute_Tau(const cDynVector6& F) = 0;

    virtual void plusEq_SddQ(cDynVector6& A) = 0;

    virtual void minusEq_SdQ_damping(cDynVector6& B, const cDynMatrix6& Ia) = 0;
    

    //---------------------------------------------------------------------------
    /*!
        0Jn = Jn = iXn^T Si = 0Xi^(-T) Si
        where 0Xi^(-T) = [ R rxR; 0 R ]
    */
    //---------------------------------------------------------------------------
    virtual void compute_Jg(const cDynTransform &globalX) = 0;

    virtual void zero_Tau() = 0;
    
    
    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
private:

    cDynVector6 _C;
    cDynVector6 _Pa;
    cDynTransform _localX;
    cDynMatrix6 _L;
};

//---------------------------------------------------------------------------

class cDynABJointFixed : public cDynABJoint
{
    //-----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //-----------------------------------------------------------------------
public:

    virtual void update_localX(const cDynTransform& home) { localX() = home; }
    virtual void plusEq_SdQ(cDynVector6& V) {}
    virtual void plusEq_V_X_SdQ(cDynVector6& C, const cDynVector6& V) {}
    virtual void compute_Dinv_and_SbarT(const cDynMatrix6& Ia) {}
    virtual void minusEq_X_SbarT_St(cDynMatrix6& L, const cDynTransform& localX) {}
    virtual void plusEq_X_SbarT_Tau(cDynVector6& Pah, const cDynTransform& localX) {}
    virtual void compute_ddQ(const cDynVector6& Pa, const cDynVector6& XAh_C) {}
    virtual void compute_ddQ(const cDynVector6& XAh_C) {}
    virtual void compute_Tau(const cDynVector6& F) {}

    virtual void plusEq_SddQ(cDynVector6& A) {}
    virtual void minusEq_SdQ_damping(cDynVector6& B, const cDynMatrix6& Ia) {}
    virtual void compute_Jg(const cDynTransform &globalX) {}
    virtual void zero_Tau() {};
};

//---------------------------------------------------------------------------

class cDynABJointFree : public cDynABJoint
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual void update_localX(const cDynTransform& home);
    virtual void plusEq_SdQ(cDynVector6& V);
    virtual void plusEq_V_X_SdQ(cDynVector6& C, const cDynVector6& V);
    virtual void compute_Dinv_and_SbarT(const cDynMatrix6& Ia);
    virtual void minusEq_X_SbarT_St(cDynMatrix6& L, const cDynTransform& localX);
    virtual void plusEq_X_SbarT_Tau(cDynVector6& Pah, const cDynTransform& localX);
    virtual void compute_ddQ(const cDynVector6& Pa, const cDynVector6& XAh_C);
    virtual void compute_ddQ(const cDynVector6& XAh_C);
    virtual void compute_Tau(const cDynVector6& F);

    virtual void plusEq_SddQ(cDynVector6& A);
    virtual void minusEq_SdQ_damping(cDynVector6& B, const cDynMatrix6& Ia);
    virtual void compute_Jg(const cDynTransform &globalX);
    virtual void zero_Tau() { _Tau.zero(); };


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
// private:

    double _motorInertia;
    double _damping;
    cDynMatrix6 _Jg;
    cDynMatrix6 _Dinv;  // LU decomposition of Ia

    cDynFrame _Q;
    cDynVector6 _dQ;
    cDynVector6 _ddQ;
    cDynVector6 _Tau;
};

//---------------------------------------------------------------------------

class cDynABJointSpherical : public cDynABJoint
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual void update_localX(const cDynTransform& home);
    virtual void plusEq_SdQ(cDynVector6& V);
    virtual void plusEq_V_X_SdQ(cDynVector6& C, const cDynVector6& V);
    virtual void compute_Dinv_and_SbarT(const cDynMatrix6& Ia);
    virtual void minusEq_X_SbarT_St(cDynMatrix6& L, const cDynTransform& localX);
    virtual void plusEq_X_SbarT_Tau(cDynVector6& Pah, const cDynTransform& localX);
    virtual void compute_ddQ(const cDynVector6& Pa, const cDynVector6& XAh_C);
    virtual void compute_ddQ(const cDynVector6& XAh_C);
    virtual void compute_Tau(const cDynVector6& F);

    virtual void plusEq_SddQ(cDynVector6& A);
    virtual void minusEq_SdQ_damping(cDynVector6& B, const cDynMatrix6& Ia);
    virtual void compute_Jg(const cDynTransform &globalX);
    virtual void zero_Tau() { _Tau.zero(); };


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
// private:
    
    double _motorInertia;
    double _damping;
    cDynMatrix3 _Jg[2];
    cDynMatrix3 _SbarT[2];
    cDynMatrix3 _Dinv;

    cDynQuaternion _Q;
    cDynVector3 _dQ;
    cDynVector3 _ddQ;
    cDynVector3 _Tau;
};

//---------------------------------------------------------------------------

class cDynABJointDOF1 : public cDynABJoint
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    virtual void update_localX(const cDynTransform& home);
    virtual void plusEq_SdQ(cDynVector6& V);
    virtual void plusEq_V_X_SdQ(cDynVector6& C, const cDynVector6& V);
    virtual void compute_Dinv_and_SbarT(const cDynMatrix6& Ia);
    virtual void minusEq_X_SbarT_St(cDynMatrix6& L, const cDynTransform& localX);
    virtual void plusEq_X_SbarT_Tau(cDynVector6& Pah, const cDynTransform& localX);
    virtual void compute_ddQ(const cDynVector6& Pa, const cDynVector6& XAh_C);
    virtual void compute_ddQ(const cDynVector6& XAh_C);
    virtual void compute_Tau(const cDynVector6& F);

    virtual void plusEq_SddQ(cDynVector6& A);
    virtual void minusEq_SdQ_damping(cDynVector6& B, const cDynMatrix6& Ia);
    virtual void compute_Jg(const cDynTransform &globalX);
    virtual void zero_Tau() { _Tau = 0; };

    virtual const cDynVector6& S() const { return _S; }
    virtual void S(const cDynVector6& unit6) { _S = unit6; }


    //-----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //-----------------------------------------------------------------------
// private:
    cDynVector6 _S;

    double _motorInertia;
    double _damping;
    cDynVector6 _Jg;
    cDynVector6 _SbarT;
    double _Dinv;

    double _Q;
    double _dQ;
    double _ddQ;
    double _Tau;
};

//---------------------------------------------------------------------------

class cDynABJointPrismatic : public cDynABJointDOF1
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    cDynABJointPrismatic(int axis)
    {
        cDynVector6 unit;
        unit.zero();
        unit[0][axis] = 1;
        S(unit);
    }
};

//---------------------------------------------------------------------------

class cDynABJointRevolute : public cDynABJointDOF1
{
    //-----------------------------------------------------------------------
    // PUBLIC METHODS:
    //-----------------------------------------------------------------------
public:

    cDynABJointRevolute(int axis)
    {
        cDynVector6 unit;
        unit.zero();
        unit[1][axis] = 1;
        S(unit);
    }
};

//---------------------------------------------------------------------------
#endif // CDynABJointH
//---------------------------------------------------------------------------

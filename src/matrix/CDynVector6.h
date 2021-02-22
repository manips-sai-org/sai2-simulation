//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynVector6H
#define CDynVector6H
//---------------------------------------------------------------------------
#include "matrix/CDynVector3.h"
#include "matrix/CDynMatrix6.h"
#include "matrix/CDynTransform.h"
#include "matrix/CDynFrame.h"
//---------------------------------------------------------------------------

class cDynVector6
{
    //----------------------------------------------------------------------
    // OPERATORS:
    //----------------------------------------------------------------------
public:

    //! Operator '='
    void operator=(const cDynVector6 &v) { vec3_[0] = v.vec3_[0]; vec3_[1] = v.vec3_[1]; }

    //! Operator '[]'
    cDynVector3 &operator[](int row) { return vec3_[row]; }
    
    //! Operator '[]'
    const cDynVector3 &operator[](int row) const { return vec3_[row]; }

    //! Operator '+='
    void operator+=(const cDynVector6 &v) { vec3_[0] += v.vec3_[0]; vec3_[1] += v.vec3_[1]; }
    
    //! Operator '-='
    void operator-=(const cDynVector6 &v) { vec3_[0] -= v.vec3_[0]; vec3_[1] -= v.vec3_[1]; }
    
    //! Operator '*='
    void operator*=(const double s) { vec3_[0] *= s; vec3_[1] *= s; }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    //! Initialize vector with zero values.
    void zero() 
    { 
        vec3_[0].zero(); 
        vec3_[1].zero(); 
    }

    //! Get element of vector.
    double &elementAt(int i) 
    { 
        return (i<3) ? vec3_[0][i] : vec3_[1][i-3]; 
    }

    //! Get element of vector
    const double &elementAt(int i) const 
    { 
        return (i<3) ? vec3_[0][i] : vec3_[1][i-3]; 
    }

    //! this = -v
    void negate(const cDynVector6 &v) 
    { 
        vec3_[0].negate(v.vec3_[0]); vec3_[1].negate(v.vec3_[1]); 
    }

    //! this = v1 + v2
    void add(const cDynVector6 &v1,const cDynVector6 &v2) 
    { 
        vec3_[0].add(v1.vec3_[0], v2.vec3_[0]); 
        vec3_[1].add(v1.vec3_[1], v2.vec3_[1]); 
    }
    
    //! this = v1 - v2
    void subtract(const cDynVector6 &v1,const cDynVector6 &v2) 
    { 
        vec3_[0].subtract(v1.vec3_[0], v2.vec3_[0]); 
        vec3_[1].subtract(v1.vec3_[1], v2.vec3_[1]); 
    }
    
    //! this[i] = v[i] * s
    void multiply(const cDynVector6 &v, const double s) 
    { 
        vec3_[0].multiply(v.vec3_[0], s); 
        vec3_[1].multiply(v.vec3_[1], s); 
    }
    
    //! this = m * v
    void multiply(const cDynMatrix6 &m, const cDynVector6 &v) 
    {
        cDynVector3 tmpV3;
        vec3_[0].multiply(m[0][0], v.vec3_[0]); tmpV3.multiply(m[0][1], v.vec3_[1]); vec3_[0] += tmpV3;
        vec3_[1].multiply(m[1][0], v.vec3_[0]); tmpV3.multiply(m[1][1], v.vec3_[1]); vec3_[1] += tmpV3;
    }
    
    //! this = m^T * v
    void transposedMultiply(const cDynMatrix6 &m, const cDynVector6 &v) 
    {
        cDynVector3 tmpV3;
        vec3_[0].transposedMultiply(m[0][0], v.vec3_[0]); tmpV3.transposedMultiply(m[1][0], v.vec3_[1]); vec3_[0] += tmpV3;
        vec3_[1].transposedMultiply(m[0][1], v.vec3_[0]); tmpV3.transposedMultiply(m[1][1], v.vec3_[1]); vec3_[1] += tmpV3;
    }
    
    //! this = T - Td
    void error(const cDynTransform &T, const cDynTransform &Td) 
    { 
        vec3_[0].subtract(T.translation(), Td.translation()); 
        vec3_[1].angularError(T.rotation(), Td.rotation()); 
    }
    
    //! this = T - Td
    void error(const cDynFrame &F, const cDynFrame &Fd)  
    { 
        vec3_[0].subtract(F.translation(), Fd.translation()); 
        vec3_[1].angularError(F.rotation(), Fd.rotation()); 
    }

    //! Dot product.
    double dot(const cDynVector6 &v) const 
    { 
        return (vec3_[0].dot(v.vec3_[0]) + vec3_[1].dot(v.vec3_[1])); 
    }

    //! this = x where y = m x
    void solve(const cDynMatrix6 &m, const cDynVector6 &y) 
    { 
        cDynMatrix6 lu; lu.luDecomp(m); backSub(lu,y); 
    }
    
    //! this = x where y = m x and m is SPD
    void solveSPD(const cDynMatrix6 &m, const cDynVector6 &y) 
    { 
        cDynMatrix6 lu; lu.luDecompSPD(m); backSubSPD(lu,y); 
    }

    //! this = x where y = LU x
    void backSub(const cDynMatrix6 &lu,const cDynVector6 &y);

    //! this = x where y = LU x and L*U is SPD
    void backSubSPD(const cDynMatrix6 &lu,const cDynVector6 &y);


    //-----------------------------------------------------------------------
    /*!
        L = X M
        X = [ R 0; dxR R ]
        Fh = X Fi
           = [ R fi ; dxR fi + R ni ]
    */
    //-----------------------------------------------------------------------
    void xform(const cDynTransform &t, const cDynVector6 &v)
    {
        vec3_[0].multiply(t.rotation(), v.vec3_[0]);
        vec3_[1].multiply(t.rotation(), v.vec3_[1]);
        cDynVector3 tmpV;
        tmpV.crossMultiply(t.translation(), vec3_[0]); 
        vec3_[1] += tmpV;
    }
    

    //-----------------------------------------------------------------------
    /*!
        Xt = [ Rt -Rtdx; 0 Rt ]
        Vi = Xt Vh
           = [ Rtvh - Rtdxwh; Rtwh ]
           = [ Rt(vh - dxwh); Rtwh ]
    */
    //-----------------------------------------------------------------------
    void xformT(const cDynTransform &t, const cDynVector6 &v) 
    {
        vec3_[0].crossMultiply(t.translation(), v.vec3_[1]);
        vec3_[1].subtract(v.vec3_[0], vec3_[0]);
        vec3_[0].transposedMultiply(t.rotation(), vec3_[1]);
        vec3_[1].transposedMultiply(t.rotation(), v.vec3_[1]);
    }
    


    //--------------------------------------------------------------------------
    /*!
        Xtinv = [ R dxR; 0 R ]
        Vi = Xtinv Vh
           = [ R vh + dxR wh;  R wh ]
    */
    //--------------------------------------------------------------------------
    void xformInvT(const cDynTransform &t, const cDynVector6 &v) 
    {
        vec3_[0].multiply(t.rotation(), v.vec3_[0]);
        vec3_[1].multiply(t.rotation(), v.vec3_[1]);
        cDynVector3 tmpV;
        tmpV.crossMultiply(t.translation(), vec3_[1]); 
        vec3_[0] += tmpV;
    }
    

    //--------------------------------------------------------------------------
    /*!
        Xinv = [ Rt 0; -Rtdx Rt ]
        Vh = Xinv Vi
           = [ Rtvi ; -Rtdxvi + Rtwi ]
           = [ Rtvi ; Rt(wi - dxvi) ]
    */
    //--------------------------------------------------------------------------
    void xformInv(const cDynTransform &t, const cDynVector6 &v) 
    {
        vec3_[1].crossMultiply(t.translation(), v.vec3_[0]);
        vec3_[0].subtract(v.vec3_[1], vec3_[1]);
        vec3_[1].transposedMultiply(t.rotation(), vec3_[0]);
        vec3_[0].transposedMultiply(t.rotation(), v.vec3_[0]);
    }


    //--------------------------------------------------------------------------
    /*!
        this = [v0;v1]X[tmp0;tmp1] = [ v1x , v0x ; 0 , v1x ] [ tmp0 ; tmp1 ]
                                   = [ v1 x tmp0 + v0 x tmp1; v1 x tmp1 ]
    */
    //--------------------------------------------------------------------------
    void crossMultiply(const cDynVector6 &v1, const cDynVector6 &v2) 
    {
        vec3_[1].crossMultiply(v1.vec3_[0], v2.vec3_[1]);
        vec3_[0].crossMultiply(v1.vec3_[1], v2.vec3_[0]);
        vec3_[0] += vec3_[1];
        vec3_[1].crossMultiply(v1.vec3_[1], v2.vec3_[1]);
    }


#ifdef CDYN_DEBUG
    void display(char *str);
#endif // CDYN_DEBUG


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:
    
    cDynVector3 vec3_[2];
};

//---------------------------------------------------------------------------
#endif // CDynVector6H
//---------------------------------------------------------------------------
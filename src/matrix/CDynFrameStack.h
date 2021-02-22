//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynFrameStackH
#define CDynFrameStackH
//---------------------------------------------------------------------------
#include <assert.h>
#include "matrix/CDynFrame.h"
#include "matrix/CDynVector3.h"
#include "matrix/CDynMatrix3.h"
#include "matrix/CDynQuaternion.h"
#include "matrix/CDynTransform.h"
//---------------------------------------------------------------------------
#define CDYN_FRAMESTACK_SIZE_DEFAULT 32
//---------------------------------------------------------------------------

class cDynFrameStack
{
    //-----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //-----------------------------------------------------------------------
public:

    //! Constructor of cDynFrameStack.    
    cDynFrameStack(const int max_size=CDYN_FRAMESTACK_SIZE_DEFAULT) : max_size_(max_size) 
    { 
        frames_ = new cDynFrame[max_size_+1]; 
        reset(); 
    }

    //! Destructor of cDynFrameStack. 
    ~cDynFrameStack() 
    { 
        delete[] frames_; 
    }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (BASIC OPERATIONS)
    //-----------------------------------------------------------------------
public:

    void reset() { top_ = EMPTY; frames_[top_].identity(); }
    const cDynFrame &top() const { return frames_[top_]; }

    int isEmpty() const { return (top_ == EMPTY); }
    int isFull() const { return (top_ == max_size_); }

    void pop() { if (!isEmpty()) top_--; }
    void push() { if (!isFull()) { frames_[top_ + 1] = frames_[top_]; top_++; } }
    void load(const cDynFrame &f) { frames_[top_] = f; }

    void multiply(const cDynFrame &f) { cDynFrame res; res.multiply(frames_[top_], f); frames_[top_] = res; }

    void inverse() { cDynFrame res; res.inverse(frames_[top_]); frames_[top_] = res; }

    void vectorTransform(const cDynVector3 &v, cDynVector3 &res) const { res.multiply(top(), v); }
    void vectorTransformInv(const cDynVector3 &v, cDynVector3 &res) const { res.inversedMultiply(top(), v); }
    void vectorRotate(const cDynVector3 &v, cDynVector3 &res) const { res.multiply(top().rotation(), v); }
    void vectorRotateInv(const cDynVector3 &v, cDynVector3 &res) const { res.inversedMultiply(top().rotation(), v); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (TRANSLATIONS)
    //-----------------------------------------------------------------------
public:

    void translate(const cDynVector3 &p) { cDynFrame f; f.identity(); f.set(p); multiply(f); }
    void translate(const double x, const double y, const double z) { cDynVector3 v; v.set(x, y, z); translate(v); }
    void translate(const cDynVector3 &axis, const double distance) { cDynVector3 v; v.multiply(axis, distance); translate(v); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (ROTATIONS)
    //-----------------------------------------------------------------------
public:

    void rotate(const cDynQuaternion &q) { cDynFrame f; f.identity(); f.set(q); multiply(f); }
    void rotate(const cDynVector3 &axis, const double angle) { cDynQuaternion q; q.set(axis,angle); rotate(q); }
    void rotate(const int axis, const double angle) { cDynQuaternion q; q.set(axis,angle); rotate(q); }


    //-----------------------------------------------------------------------
    // PUBLIC METHODS: (ROBOTICS)
    //-----------------------------------------------------------------------
public:

    //! Denavit-Hartenberg.
    void dh(const double alpha, const double a, const double d, const double theta) 
    {
        cDynTransform T;
        T.set(alpha, a, d, theta);
        cDynFrame F;
        F.set(T);
        multiply(F);
    }

    //! Screw.
    void screw(const cDynVector3 &axis, const double pitch, const double angle) 
    {
        cDynTransform T;
        T.set(axis, pitch, angle);
        cDynFrame F;
        F.set(T);
        multiply(F);
    }

    //! Axis angle.
    void axis2(const cDynAxis axis,const cDynVector3& a, const cDynVector3& b) 
    {
        int ia= (axis + 1)%3;
        int ib= (axis + 2)%3;
        int ic= axis;
        cDynVector3 v[3];
        v[ic].crossMultiply(a,b);
        assert(v[ic].magnitude() > 1e-8); v[ic].normalize();
        v[ib].crossMultiply(v[ic],a); v[ib].normalize();
        v[ia]=a; v[ia].normalize();
        cDynMatrix3 mat;
        mat.set(
            v[ia][0], v[ib][0], v[ic][0],
            v[ia][1], v[ib][1], v[ic][1],
            v[ia][2], v[ib][2], v[ic][2]
        );
        cDynQuaternion q; q.set(mat);
        rotate(q);
    }


    //-----------------------------------------------------------------------
    // PRIVATE METHODS:
    //-----------------------------------------------------------------------
private:

    cDynFrame *frames_;
    int top_;
    int max_size_;

    enum {EMPTY = 0};

    // Copy is not allowed
    cDynFrameStack(const cDynFrameStack &) {}
};

//---------------------------------------------------------------------------
#endif // CDynFrameStackH_
//---------------------------------------------------------------------------
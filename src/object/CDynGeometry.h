//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynGeometryH
#define CDynGeometryH
//---------------------------------------------------------------------------
#include "object/CDynPrimitive.h"
//---------------------------------------------------------------------------
class cDynMaterial;
class cDynPrim;
class cDynBSphere;
class cDynBoundRecord;
//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------

class cDynGeometryNode
{
    friend class cDynGeometry;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
private:

    //! Constructor of cDynGeometryNode.    
    cDynGeometryNode() { reset(); rc_=1; }

    //! Destructor of cDynGeometryNode.
    ~cDynGeometryNode() { remove(); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

    const cDynBSphere *bs() const { return(bs_); }
    const cDynPrim *prim() const { return(prim_); }
    const cDynPrim *bound() const { return(bound_); }

    void reference() { rc_++; }
    void unreference() { if (--rc_ == 0) delete this; }


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:
    
    void reset() { prim_=NULL;bs_=NULL;bound_=NULL; }
    void remove();
    void build();
    void prim(cDynPrim *p);
    void bound(cDynPrim *b);
    void bsphere(cDynBSphere *bs);
    
    cDynPrim *prim_;
    cDynBSphere *bs_;
    cDynPrim *bound_;

    int rc_;
};

//---------------------------------------------------------------------------
#endif // CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
/*! 
    \class cDynGeometry
    \brief A class to define geometry used for collision testing
 
    This class permits the specification of object geometry used for 
    collision testing. Primitives specified by begin(cDynPrimitiveType)
    are not valid until full specification is completed and end() 
    is called.
 
    \warning In normal use cDynGeometry should only be specified as part of 
    a part of cDynObject::geometry.
    
    \note This class does NOT alter the dynamic mass and inertia properties 
    defined for the object. 
 
   \sa cDynObject, cDynPrimitive
*/
//---------------------------------------------------------------------------

class cDynGeometry
{
    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //! Constructor of cDynGeometry.   
    cDynGeometry();
    cDynGeometry(cDynGeometry& src);
    cDynGeometry& operator=(cDynGeometry& src);

    //! Destructor of cDynGeometry.
    ~cDynGeometry();


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (DEFINING AND CLOSING PRIMITIVES)
    //----------------------------------------------------------------------
public:

    //----------------------------------------------------------------------
    /*!
        Opens a cDynPrimitive object to permit the specification of vertices
        and properties of a base primitive or group of like primitives. begin accepts
        a single argument that specifies which of eleven ways the vertices
        are interpreted.  Taking n as an integer count starting at one,
        and N as the total number of vertices specified, the interpretations
        are as follows:
        
        The primitives specified by cDynPrimitive do not become valid until the primitive
        is closed by cDynGeometry::end(cDynPrimitive *) or cDynPrimitive::end().
        
        \code
        
        // specify a cube
        cDynPrimitive* p=obj->geometry.begin(CDYN_HULL);
            p->radius(1e-3); 
            p->error(0.5e-3);
            p->vertex(-w,-l,-h);
            p->vertex( w,-l,-h);
            p->vertex( w, l,-h);
            p->vertex(-w, l,-h);
            p->vertex(-w,-l, h);
            p->vertex( w,-l, h);
            p->vertex( w, l, h);
            p->vertex(-w, l, h);
        p->end();
        obj->geometry.end();
        \endcode
    */
    //----------------------------------------------------------------------
    cDynPrimitive* begin(const cDynPrimitiveType type);


    //----------------------------------------------------------------------
    /*!
        Ends specification of a primitive. Equivalent to \a p->end()
    */
    //----------------------------------------------------------------------
    void end(cDynPrimitive* p);


    //----------------------------------------------------------------------
    /*!
        Closes specification of an objects geometry and makes it available 
        for collision testing.
    */
    //----------------------------------------------------------------------
    void end(void);


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (MATERIAL)
    //----------------------------------------------------------------------
public:

    //----------------------------------------------------------------------
    /*!
        Set default material to be used for primitives defined by begin()
    
        \param m default material.
    */
    //----------------------------------------------------------------------
    void material(cDynMaterial* m) { material_=m; };


    //----------------------------------------------------------------------
    /*!
        Returns the default material specified by material(cDynMaterial* m),
        \c NULL otherwise.
        
        \return  Return currently active material.
    */
    //----------------------------------------------------------------------
    cDynMaterial* material() { return(material_); };


    //----------------------------------------------------------------------
    // PUBLIC METHODS: (SPECIAL)
    //----------------------------------------------------------------------
public:

    //----------------------------------------------------------------------
    /*!
        Set the frame of reference to be used when specifying the vertices 
        of a primitive. By default the current frame is cDynObject::frame. 
        \c NULL specifies the identity matrix.

        \param fs Frame of reference used for all future primitives 
                  specified by begin()
    */
    //----------------------------------------------------------------------
    void frame(cDynFrameStack *fs) { fs_=fs; }


    //----------------------------------------------------------------------
    /*!
        Return the current active frame of reference.
    */
    //----------------------------------------------------------------------
    cDynFrameStack* frame() { return(fs_); }


    //----------------------------------------------------------------------
    // PUBLIC METHODS:
    //----------------------------------------------------------------------
public:

#ifndef CDYN_DONT_DOCUMENT
    //! Used to fetch information for collision testing
    const cDynBSphere *bs() { return(ptr_->bs()); }
    const cDynPrim *prim() { return(ptr_->prim()); }
    const cDynPrim *bound() { return(ptr_->bound()); }
    cDynBoundRecord *box() { return(box_); }
    void box(cDynBoundRecord* b) { box_=b; }

    //! Only used by cDynPrimitive
    void prim(cDynPrim *p) { ptr_->prim(p); }  
    void bound(cDynPrim *p) { ptr_->bound(p); } 
    void bsphere(cDynBSphere *bs) { ptr_->bsphere(bs); }  
#endif


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    //! Build sphere hierarchy (on close)
    void hierarchy(void);	

    cDynFrameStack* fs_;
    cDynMaterial *material_;

    //! Structure used to store entities for an axis aligned bounding box
    cDynBoundRecord* box_;

    //! Prim, Bound and Binary Sphere List
    cDynGeometryNode* ptr_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
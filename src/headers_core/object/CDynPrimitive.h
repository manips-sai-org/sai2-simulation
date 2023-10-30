//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynPrimitiveH
#define CDynPrimitiveH
//---------------------------------------------------------------------------
const double   DEPRIMITIVE_MIN_ERROR = 1e-8f;
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/*!
CDYN_POINTS:
Each vertex represents a single point.  Vertex \a n defines point \a n. 
\a N points are specified. Setting a radius() defines a sphere.

CDYN_LINES:
Each pair of vertices represents an independent line segment.  
Vertices \a 2n-1 and \a 2n define line \a n.  \a N/2 lines are specified. 
Setting a radius() defines a capsule.

CDYN_LINE_STRIP:
Defines a connected group of line segments from the first vertex to the last.  
Vertices \a n and \a n+1 define line \a n.  \a N-1 lines are specified.

CDYN_LINE_LOOP:
Defines a connected group of line segments from the first vertex to the 
last, then back to the first.  Vertices \a n and \a n+1 define line \a n.  
The last line, however, is defined by vertices \a N and \a 1.  \a N lines 
are specified.

CDYN_TRIANGLES:
Each triplet of vertices define an independent triangle.  Vertices \a 3n-2, 
\a 3n-1, and \a 3n define triangle \a n.  \a N/3 triangles are specified.

CDYN_TRIANGLE_STRIP:
Define a connected group of triangles.  One triangle is defined for each 
vertex presented after the first two vertices.  For odd \a n, vertices 
\a n, \a n+1, and \a n+2 define triangle \a n.  For even \a n, vertices 
\a n+1, \a n, and \a n+2 define triangle n.  N-2 triangles are specified.

CDYN_TRIANGLE_FAN:
Define a connected group of triangles.  One triangle is defined for each 
vertex presented after the first two vertices.  Vertices \a 1, \a n+1, 
and \a n+2 define triangle \a n.  \a N-2 triangles are drawn.

CDYN_QUADS:
Each group of four vertices defines an independent quadrilateral.  
Vertices \a 4n-3, \a 4n-2, \a 4n-1, and \a 4n define quadrilateral \a n.  
\a N/4 quadrilaterals are specified.

CDYN_QUAD_STRIP:
Defines a connected group of quadrilaterals.  One quadrilateral is defined 
for each pair of vertices presented after the first pair.  Vertices 
\a 2n-1, \a 2n, \a 2n+2, and \a 2n+1 define quadrilateral \a n.  
\a N/2-1 quadrilaterals are drawn.  \note The order in which vertices 
are used to construct a quadrilateral from strip data is different from 
that used with independent data.

CDYN_POLYGON:
Defines a single, convex polygon.  Vertices \a 1 through \a N define 
this polygon.

CDYN_POLYGON_CONCAVE:
\a not \a supported \a at \a this \a time

CDYN_HULL:
Defines a single convex polyhedra consisting of the convex hull of 
all the vertices \a through \a N.

CDYN_BOUND:
Defines a single convex bounding volume consisting of the convex hull 
of all the vertices \a 1 through \a N. Only one bound may be specified per 
object. If more then one bound is specified the last one is used.
*/
//---------------------------------------------------------------------------

enum cDynPrimitiveType 
{
    CDYN_POINTS, 	
    CDYN_LINES,  	
    CDYN_LINE_STRIP,	
    CDYN_LINE_LOOP,	
    CDYN_TRIANGLES,	
    CDYN_TRIANGLE_STRIP,
    CDYN_TRIANGLE_FAN,	
    CDYN_QUADS,	
    CDYN_QUAD_STRIP,	
    CDYN_POLYGON,	
    CDYN_POLYGON_CONCAVE,	
    CDYN_HULL,	
    CDYN_BOUND	
};

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
class cDynGeometry;
class cDynMaterial;
//---------------------------------------------------------------------------

class cDynPrimitive
{
    friend class cDynGeometry;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
private:
    //! Constructor of cDynPrimitive.  
    cDynPrimitive(cDynGeometry* geom, const cDynPrimitiveType type);

public:
    //! Destructor of cDynPrimitive.
    ~cDynPrimitive();


    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------
public:

    //! Name vertex definition methods.
    void vertex(const cDynVector3& v);
    void vertex(const double x, const double y, const double z);
    void vertex(const double x, const double y);
    void vertex(const double v[3]);

    //! Define primitive as all points with \a r distance of basic primitive.
    void radius(const double r);

    //! Error tolerance to use to determine when primitive is in contact.
    void error(const double err);

    //! Bind material property to primitive or primitive group.
    void material(cDynMaterial* m) { material_ = m; };

    //! Return material property of primitive or group
    cDynMaterial* material() const { return(material_); };

    //! User defined data methods
    char *data() const { return(data_); }
    void data(char* d) { data_=d; }

    //! Close a primitive, and add its geometry to object geometry (cDynPrimitive deleted). 
    void end();

    //! Return geometry to which primitive is bound.
    void geometry(cDynGeometry* geom) { geom_=geom; }

    //! Return type of primitive or group of primitives being defined.
    void type(cDynPrimitiveType type) { type_=type; }


    //----------------------------------------------------------------------
    // PRIVATE METHODS:
    //----------------------------------------------------------------------
private:

    // Used by cDynGeometry::end()
    void convert(void);

    // Primitives constructors
    void point(int i);
    void line(int i0, int i1);
    void triangle(int i0, int i1, int i2);
    void quad(int i0, int i1, int i2, int i3);
    void polygon(int istart, int iend);
    void hull(int istart, int iend);
    void bound(int istart, int iend);
    
    cDynGeometry* geom_;
    cDynPrimitiveType type_;

    //! Radius of primitive.
    double radius_;	

    //! Error bound.
    double error_;		

    //! Material properties
    cDynMaterial* material_;

    //! User data
    char* data_;

    //! Vertex information
    cDynVector3 *pvertex_;
    int nv_;
    int nvallocated_;
    int nchunk_;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
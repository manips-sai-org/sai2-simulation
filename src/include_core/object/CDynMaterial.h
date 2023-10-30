//===========================================================================
/*
    This file is part of the dynamics library.
    Copyright (C) 2014, Artificial Intelligence Laboratory,
    Stanford University. All rights reserved.

    \version   $MAJOR.$MINOR.$RELEASE $Rev: 1242 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDynMaterialH
#define CDynMaterialH
//---------------------------------------------------------------------------
#include "matrix/CDynMathDefn.h"
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
/*!
CDYN_MODEL_MIN:
The given property is taken to be the minimum of the default properties 
of each material.

CDYN_MODEL_MEAN:
The given property is taken to be the average of the default properties 
of each material.

CDYN_MODEL_MAX:
The given property is taken to be the maximum of the default properties 
of each material.
*/
//---------------------------------------------------------------------------

enum cDynMaterialModel 
{
    CDYN_MODEL_MIN,	 
    CDYN_MODEL_MEAN,  
    CDYN_MODEL_MAX   
};


//---------------------------------------------------------------------------
/*!
CDYN_FRICTION_STATIC:
Sets the coefficient of static friction
 
CDYN_FRICTION_DYNAMIC:
Sets the coefficient of dynamic friction
 
CDYN_FRICTION_VISCOUS:
Sets the coefficient of viscous friction

CDYN_FRICTION_GRIP:
Sets max impulse friction force that can be used to maintain a firm contact
*/
//---------------------------------------------------------------------------

enum cDynFrictionType  
{
    CDYN_FRICTION_STATIC=0,   
    CDYN_FRICTION_DYNAMIC=1,  
    CDYN_FRICTION_VISCOUS=2,  
    CDYN_FRICTION_GRIP=3,     
    CDYN_VELOCITY_LIMIT=4
};


//---------------------------------------------------------------------------
const int CDYN_MATERIAL_FRICTION_MAX=5;
const int CDYN_MATERIAL_DEFAULT_SIZE=13;
//---------------------------------------------------------------------------
class cDynMaterialPair;
//---------------------------------------------------------------------------

class cDynMaterial
{

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
public:

    //--------------------------------------------------------------------------
    /*!
        \brief
        Constructor used for creating a new material property

        \param      size  The default hash table size for looking up bindings 
                          between pairs of materials
        \param      data  User defined data
    */
    //--------------------------------------------------------------------------
    cDynMaterial(const int size=CDYN_MATERIAL_DEFAULT_SIZE, char *data=NULL);

    //! Destructor of cDynMaterial.
    ~cDynMaterial();
    
    
    //----------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //----------------------------------------------------------------------   
public:

    //! set user defined data
    char *data() const { return(data_); }

    //! return user defined data
    void data(char* d) { data_=d; }

    //! Remove all bindings between material and material \a m, resort to defaults.
    void unbind(cDynMaterial* m);


    //--------------------------------------------------------------------------
    /*!
        Set the elasticity between a pair of materials. \a m=NULL sets default 
        elasticity value to use if no binding exists between a pair of materials.
        
        \a e=0 indicates a completely inelastic collision.
        \a e=1 indicates a completely elastic collision.
        
        Values greater then 1 are permitted but will add energy to system
    */
    //--------------------------------------------------------------------------
    void epsilon(const double e,cDynMaterial* m=NULL);

    //! get elasticity between a pair of materials. \a m=NULL for default elasticity.
    double epsilon(cDynMaterial* m=NULL);


    //--------------------------------------------------------------------------
    /*!
        Set default friction properties or properties between a pair of materials.
        
        \param type The \a type of friction parameter can be one of the following:
        \param type     cDynFrictionType type.
        \param u        Value for given type
        \param m        Material pair \a (this,m) for which this property holds. 
                        \a NULL indicates default material
    */
    //--------------------------------------------------------------------------
    void friction(const cDynFrictionType type, const double u,cDynMaterial* m=NULL);

    //! Return the current friction value of type \a type.
    double friction(const cDynFrictionType type, cDynMaterial* m=NULL);


    //--------------------------------------------------------------------------
    /*!
        Set model to be used for determining the friction properties of 
        unspecified  pairs. If a material binding does not exist between a pair 
        of materials the materials default values are used. 
        
        \param  model The \a model used to determine the friction properties of 
                      a given pair of materials is as follows:

        \warning    If a pair of materials have different material models then 
        the resulting friction model is undetermined.
    */
    //--------------------------------------------------------------------------
    void model(const cDynMaterialModel model) { model_=model; }


    //--------------------------------------------------------------------------
    /*!
        Return the current model used for determining properties between
        pairs of material for which no direct specification exists.
    */
    //--------------------------------------------------------------------------
    cDynMaterialModel model() { return(model_); }


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:

    int h(const cDynMaterial* m);
    cDynMaterial* lookup(const cDynMaterial* m);
    cDynMaterial* create(cDynMaterial* m);

    char* data_;
    double epsilon_;
    double vlimit_;
    double friction_[CDYN_MATERIAL_FRICTION_MAX];

    cDynMaterialPair** hash_;
    int size_;


    //----------------------------------------------------------------------
    // PRIVATE STATIC MEMBERS:
    //----------------------------------------------------------------------
private:

    static cDynMaterialModel model_;
};


//---------------------------------------------------------------------------
#ifndef CDYN_DONT_DOCUMENT
//---------------------------------------------------------------------------
class cDynMaterialPair
{
public:
    friend class cDynMaterial;

    //----------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //----------------------------------------------------------------------
private:
    cDynMaterialPair() {};


    //----------------------------------------------------------------------
    // PRIVATE MEMBERS:
    //----------------------------------------------------------------------
private:
    cDynMaterial* a_;
    cDynMaterial* r_;

    cDynMaterialPair* next_;
};
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
#ifndef ATG_SCS_2D_MY_DEMO_H
#define ATG_SCS_2D_MY_DEMO_H

#include "demo.h"

#include "bar_object.h"
#include "disk_object.h"
#include "plotter.h"
#include "constant_rotation_constraint.h"
#include "scs.h"

class MyDemo : public Demo {
    public:
        MyDemo();
        virtual ~MyDemo();
    
        virtual void initialize();
        virtual void process(float dt);
        virtual void render();
    
    protected:
        atg_scs::OptimizedNsvRigidBodySystem m_rigidBodySystem;

        DiskObject *m_end;
        BarObject* m_bar;
        Plotter *m_plotter;
        Plotter* m_plotter2;
        ConstantRotationConstraint *m_motor;

        bool m_motorOn;
};

#endif /* ATG_SCS_2D_MY_DEMO_H */

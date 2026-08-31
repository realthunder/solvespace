#ifndef _SLVS_SWIG_
#define _SLVS_SWIG_

#include <stdexcept>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <map>
#include "slvs.h"

class System {
private:
    std::map<Slvs_hParam,Slvs_Param> ParamMap;
    std::map<Slvs_hConstraint,Slvs_Constraint> ConstraintMap;
    std::map<Slvs_hEntity,Slvs_Entity> EntityMap;
    std::vector<Slvs_Param> Param;
    std::vector<Slvs_Entity> Entity;
    std::vector<Slvs_Constraint> Constraint;
    std::vector<Slvs_hParam> Dragged;

public:
    std::vector<Slvs_hConstraint> Failed;
    Slvs_hGroup GroupHandle;
    Slvs_hParam ParamHandle;
    Slvs_hEntity EntityHandle;
    Slvs_hConstraint ConstraintHandle;
    int Dof;

    System() {
        reset();
    }

    void clearDragged() {
        Dragged.clear();
    }

    void addDragged(Slvs_hParam p) {
        if(!p) return;
        for(size_t i = 0; i < Dragged.size(); ++i)
            if(Dragged[i] == p) return;
        if(Dragged.size() >= 4) return;
        Dragged.push_back(p);
    }

    void markDraggedPoint(Slvs_hEntity pt) {
        addDragged(getEntityParam(pt, 0));
        addDragged(getEntityParam(pt, 1));
    }

    void reset() {
        ParamMap.clear();
        EntityMap.clear();
        ConstraintMap.clear();
        Failed.clear();
        Dragged.clear();
        GroupHandle = 1;
        ParamHandle = 0;
        EntityHandle = 0;
        ConstraintHandle = 0;
        Dof = -1;
    }

    int solve(Slvs_hGroup group=0, bool reportFailed=false, bool findFreeParams=false) {
        Slvs_System sys = {};
#define SLVS_PREPARE(_name,_n,_ns) \
        _name.clear();\
        _name.reserve(_name##Map.size());\
        for(const auto &v : _name##Map)\
            _name.emplace_back(v.second);\
        sys._n = &_name[0];\
        sys._ns = (int)_name.size();
        SLVS_PREPARE(Param,param,params)
        SLVS_PREPARE(Entity,entity,entities)
        SLVS_PREPARE(Constraint,constraint,constraints)

        Failed.clear();
        if(reportFailed) {
            Failed.resize(Constraint.size());
            sys.faileds = (int)Failed.size();
            sys.failed = &Failed[0];
            sys.calculateFaileds = 1;
        }
        sys.findFreeParams = findFreeParams;

        for(size_t i = 0; i < 4; ++i)
            sys.dragged[i] = (i < Dragged.size()) ? Dragged[i] : 0;

        if(!group) group = GroupHandle;

        try {
            Slvs_Solve(&sys, group);
        }catch(std::exception &) {
            Failed.clear();
            Dof = -1;
            throw;
        }
        Dof = sys.dof;

        int i = 0;
        for(auto &v : ParamMap)
            v.second.val = Param[i++].val;
        if(reportFailed)
            Failed.resize(sys.faileds);
        return sys.result;
    }

#define SLVS_ACCESSOR(_name) \
    const Slvs_##_name &get##_name(Slvs_h##_name h) const \
    {\
        auto it = _name##Map.find(h);\
        if(it==_name##Map.end())\
            throw std::invalid_argument(#_name " handle not found");\
        return it->second;\
    }\
    void remove##_name(Slvs_h##_name h) \
    {\
        auto it = _name##Map.find(h);\
        if(it==_name##Map.end())\
            throw std::invalid_argument(#_name "handle not found");\
        _name##Map.erase(it);\
    }\
    Slvs_h##_name add##_name(const Slvs_##_name &v, bool overwrite=false) \
    {\
        if(!v.h)\
            throw std::invalid_argument("invalid " #_name " handle");\
        if(!v.group)\
            throw std::invalid_argument("invalid group");\
        auto it = _name##Map.find(v.h);\
        if(it!=_name##Map.end()) {\
            if(!overwrite)\
                throw std::invalid_argument("duplicate " #_name " handle");\
            it->second = v;\
        }else\
            _name##Map[v.h] = v;\
        return v.h;\
    }

    SLVS_ACCESSOR(Param)
    SLVS_ACCESSOR(Constraint)
    SLVS_ACCESSOR(Entity)

    Slvs_hParam getEntityParam(Slvs_hEntity h, int idx) const
    {
#define SLVS_GET_ENTITY(_name,_idx) \
        if(idx<0 || idx>=_idx)\
            throw(std::invalid_argument("invalid " #_name " index"));\
        auto it = EntityMap.find(h);\
        if(it==EntityMap.end())\
            throw std::invalid_argument("Entity handle not found")

        SLVS_GET_ENTITY(param,7);
        return it->second.param[idx];
    }

    void setEntityParam(Slvs_hEntity h, int idx, Slvs_hParam hParam)
    {
        SLVS_GET_ENTITY(param,7);
        it->second.param[idx] = hParam;
    }

    Slvs_hParam getEntityPoint(Slvs_hEntity h, int idx) const
    {
        SLVS_GET_ENTITY(point,4);
        return it->second.point[idx];
    }

    void setEntityPoint(Slvs_hEntity h, int idx, Slvs_hEntity hEntity)
    {
        SLVS_GET_ENTITY(point,4);
        it->second.point[idx] = hEntity;
    }

    Slvs_hParam addParamV(double val, Slvs_hGroup group=0, Slvs_hParam h=0) 
    {
#define SLVS_ADD(_name,...) \
        if(!h) h = ++_name##Handle;\
        if(!group) group=GroupHandle;\
        auto _var = Slvs_Make##_name(h,group, ## __VA_ARGS__);\
        return add##_name(_var);\

        SLVS_ADD(Param,val);
    }

    Slvs_hEntity addPoint2d(Slvs_hEntity wrkpln,
                            Slvs_hParam u, Slvs_hParam v,
                            Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
#define SLVS_ADD_ENTITY(_name,...) \
        if(!h) h = ++EntityHandle;\
        if(!group) group=GroupHandle;\
        auto _var = Slvs_Make##_name(h,group, ## __VA_ARGS__);\
        return addEntity(_var);

        SLVS_ADD_ENTITY(Point2d,wrkpln,u,v);
    }

    Slvs_hEntity addPoint2dV(Slvs_hEntity wrkpln, double u, double v,
                             Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        return addPoint2d(wrkpln,addParamV(u,group),addParamV(v,group),group,h);
    }

    Slvs_hEntity addPoint3d(Slvs_hParam x, Slvs_hParam y, Slvs_hParam z,
                             Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(Point3d,x,y,z);
    }

    Slvs_hEntity addPoint3dV(double x, double y, double z,
                             Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        return addPoint3d(addParamV(x,group),addParamV(y,group),
                addParamV(z,group),group,h);
    }

    Slvs_hEntity addNormal3d(Slvs_hParam qw, Slvs_hParam qx,
                             Slvs_hParam qy, Slvs_hParam qz,
                             Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(Normal3d,qw,qx,qy,qz);
    }

    Slvs_hEntity addNormal3dV(double qw, double qx, double qy, double qz,
                             Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        return addNormal3d(addParamV(qw,group),addParamV(qx,group),
                addParamV(qy,group),addParamV(qz,group),group,h);
    }

    Slvs_hEntity addNormal2d(Slvs_hEntity wrkpln, 
            Slvs_hGroup group=0, Slvs_hEntity h=0) 
    {
        SLVS_ADD_ENTITY(Normal2d,wrkpln);
    }

    Slvs_hEntity addDistance(Slvs_hParam d,Slvs_hGroup group=0,Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(Distance,0,d);
    }

    Slvs_hEntity addDistanceV(double d, Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        return addDistance(addParamV(d,group),group,h);
    }

    Slvs_hEntity addLineSegment(Slvs_hEntity p1, Slvs_hEntity p2, 
            Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(LineSegment,0,p1,p2);
    }

    Slvs_hEntity addCubic(Slvs_hEntity wrkpln,
                          Slvs_hEntity p1, Slvs_hEntity p2,
                          Slvs_hEntity p3, Slvs_hEntity p4,
                          Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(Cubic,wrkpln,p1,p2,p3,p4);
    }

    Slvs_hEntity addArcOfCircle(Slvs_hEntity wrkpln, Slvs_hEntity center,
                    Slvs_hEntity start, Slvs_hEntity end,
                    Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        auto &w = getEntity(wrkpln);
        SLVS_ADD_ENTITY(ArcOfCircle,wrkpln,w.normal,center,start,end);
    }

    Slvs_hEntity addCircle(Slvs_hEntity center, Slvs_hEntity normal, 
            Slvs_hEntity radius, Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(Circle,0,center,normal,radius);
    }

    Slvs_hEntity addCircleV(Slvs_hEntity center, Slvs_hEntity normal, 
                    double radius, Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        return addCircle(center,normal,addDistanceV(radius,group,h),group,h);
    }

    Slvs_hEntity addWorkplane(Slvs_hEntity origin, Slvs_hEntity normal,
                              Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(Workplane,origin,normal);
    }

    Slvs_hEntity addTransform(Slvs_hEntity src,  
            Slvs_hParam dx, Slvs_hParam dy, Slvs_hParam dz,
            Slvs_hParam qw, Slvs_hParam qx, Slvs_hParam qy, Slvs_hParam qz,
            bool asAxisAngle=false, double scale=1.0, int timesApplied=0, 
            Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(Transform,src,dx,dy,dz,qw,qx,qy,qz,
                0,asAxisAngle?1:0,scale,timesApplied);
    }

    Slvs_hEntity addTranslate(Slvs_hEntity src,  
            Slvs_hParam dx, Slvs_hParam dy, Slvs_hParam dz,
            double scale=1.0, int timesApplied=0, 
            Slvs_hGroup group=0, Slvs_hEntity h=0)
    {
        SLVS_ADD_ENTITY(Transform,src,dx,dy,dz,0,0,0,0,1,0,scale,timesApplied);
    }

    Slvs_hConstraint addConstraintV(int tp, Slvs_hEntity wrkpln, double v,
                                    Slvs_hEntity p1, Slvs_hEntity p2,
                                    Slvs_hEntity e1, Slvs_hEntity e2,
                                    Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        SLVS_ADD(Constraint,tp,wrkpln,v,p1,p2,e1,e2);
    }

    Slvs_hConstraint addPointsDistance(double d,
                    Slvs_hEntity p1, Slvs_hEntity p2, Slvs_hEntity wrkpln=0, 
                    Slvs_hGroup group=0, Slvs_hConstraint h=0) 
    {
        return addConstraintV(SLVS_C_PT_PT_DISTANCE,wrkpln,d,p1,p2,0,0,group,h);
    }

    Slvs_hConstraint addPointsProjectDistance(double d,
                    Slvs_hEntity p1, Slvs_hEntity p2, Slvs_hEntity line, 
                    Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_PROJ_PT_DISTANCE,0,d,p1,p2,line,0,group,h);
    }

    Slvs_hConstraint addPointsCoincident(Slvs_hEntity p1, Slvs_hEntity p2,
                Slvs_hEntity wrkpln=0, Slvs_hGroup group=0, Slvs_hConstraint h=0) 
    {
        return addConstraintV(SLVS_C_POINTS_COINCIDENT,wrkpln,0,p1,p2,0,0,group,h);
    }

    Slvs_hConstraint addPointPlaneDistance(double d, Slvs_hEntity pt, 
            Slvs_hEntity pln, Slvs_hGroup group=0, Slvs_hConstraint h=0) 
    {
        return addConstraintV(SLVS_C_PT_PLANE_DISTANCE, 0,d,pt,0,pln,0,group,h);
    }

    Slvs_hConstraint addPointLineDistance(double d, Slvs_hEntity pt, 
                    Slvs_hEntity line, Slvs_hEntity wrkpln=0, 
                    Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_PT_LINE_DISTANCE,wrkpln,d,pt,0,line,0,group,h);
    }

    Slvs_hConstraint addPointInPlane(Slvs_hEntity pt, Slvs_hEntity pln,
                                Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_PT_IN_PLANE,0,0,pt,0,pln,0,group,h);
    }

    Slvs_hConstraint addPointOnLine(Slvs_hEntity pt, Slvs_hEntity line,
            Slvs_hEntity wrkpln=0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_PT_ON_LINE,wrkpln,0,pt,0,line,0,group,h);
    }

    Slvs_hConstraint addEqualLength(Slvs_hEntity l1, Slvs_hEntity l2,
            Slvs_hEntity wrkpln = 0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_EQUAL_LENGTH_LINES,wrkpln,0,0,0,l1,l2,group,h);
    }

    Slvs_hConstraint addLengthRatio(double ratio, Slvs_hEntity l1, Slvs_hEntity l2,
            Slvs_hEntity wrkpln = 0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_LENGTH_RATIO,wrkpln,ratio,0,0,l1,l2,group,h);
    }

    Slvs_hConstraint addLengthDifference(double diff, Slvs_hEntity l1, 
                            Slvs_hEntity l2, Slvs_hEntity wrkpln=0, 
                            Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_LENGTH_DIFFERENCE,wrkpln,diff,0,0,l1,l2,group,h);
    }

    Slvs_hConstraint addEqualLengthPointLineDistance(Slvs_hEntity pt, 
                        Slvs_hEntity l1, Slvs_hEntity l2, Slvs_hEntity wrkpln=0, 
                        Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_EQ_LEN_PT_LINE_D, wrkpln,0,pt,0,l1,l2,group,h);
    }

    Slvs_hConstraint addEqualPointLineDistance(Slvs_hEntity p1,
            Slvs_hEntity l1, Slvs_hEntity p2, Slvs_hEntity l2, 
            Slvs_hEntity wrkpln=0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_EQ_PT_LN_DISTANCES,
                wrkpln,0,p1,p2,l1,l2,group,h);
    }

    Slvs_hConstraint addEqualAngle(bool supplement,
        Slvs_hEntity l1,Slvs_hEntity l2, Slvs_hEntity l3, Slvs_hEntity l4,
        Slvs_hEntity wrkpln=0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
#define SLVS_INIT_CSTR(...) \
        if(!h) h = ++ConstraintHandle;\
        if(!group) group=GroupHandle;\
        auto _var = Slvs_MakeConstraint(h,group, ## __VA_ARGS__);\

        SLVS_INIT_CSTR(SLVS_C_EQUAL_ANGLE,wrkpln,0,0,0,l1,l2);
        _var.entityC = l3;
        _var.entityD = l4;
        _var.other = supplement?1:0;
        return addConstraint(_var);
    }

    Slvs_hConstraint addEqualLineArcLength(Slvs_hEntity line, Slvs_hEntity arc,
            Slvs_hEntity wrkpln=0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_EQUAL_LINE_ARC_LEN,
                wrkpln,0,0,0,line,arc,group,h);
    }

    Slvs_hConstraint addSymmetric(Slvs_hEntity p1, Slvs_hEntity p2,
                            Slvs_hEntity pln, Slvs_hEntity wrkpln=0, 
                            Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_SYMMETRIC,wrkpln,0,p1,p2,pln,0,group,h);
    }

    Slvs_hConstraint addSymmetricHorizontal(Slvs_hEntity p1, Slvs_hEntity p2,
            Slvs_hEntity wrkpln, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_SYMMETRIC_HORIZ,wrkpln,0,p1,p2,0,0,group,h);
    }

    Slvs_hConstraint addSymmetricVertical(Slvs_hEntity p1, Slvs_hEntity p2,
            Slvs_hEntity wrkpln, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_SYMMETRIC_VERT,wrkpln,0,p1,p2,0,0,group,h);
    }

    Slvs_hConstraint addSymmetricLine(Slvs_hEntity p1, Slvs_hEntity p2,
                                Slvs_hEntity line, Slvs_hEntity wrkpln, 
                                Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_SYMMETRIC_VERT,wrkpln,0,p1,p2,line,0,group,h);
    }

    Slvs_hConstraint addMidPoint(Slvs_hEntity pt, Slvs_hEntity line,
            Slvs_hEntity wrkpln=0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_AT_MIDPOINT,wrkpln,0,pt,0,line,0,group,h);
    }

    Slvs_hConstraint addPointsHorizontal(Slvs_hEntity p1, Slvs_hEntity p2,
            Slvs_hEntity wrkpln, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_HORIZONTAL,wrkpln,0,p1,p2,0,0,group,h);
    }

    Slvs_hConstraint addPointsVertical(Slvs_hEntity p1, Slvs_hEntity p2,
            Slvs_hEntity wrkpln, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_VERTICAL,wrkpln,0,p1,p2,0,0,group,h);
    }

    Slvs_hConstraint addLineHorizontal(Slvs_hEntity line, Slvs_hEntity wrkpln, 
                                       Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_HORIZONTAL,wrkpln,0,0,0,line,0,group,h);
    }

    Slvs_hConstraint addLineVertical(Slvs_hEntity line, Slvs_hEntity wrkpln, 
                                    Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_VERTICAL,wrkpln,0,0,0,line,0,group,h);
    }

    Slvs_hConstraint addDiameter(double d, Slvs_hEntity c, 
                    Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_DIAMETER,0,d,0,0,c,0,group,h);
    }

    Slvs_hConstraint addPointOnCircle(Slvs_hEntity pt, Slvs_hEntity circle,
                                      Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_PT_ON_CIRCLE,0,0,pt,0,circle,0,group,h);
    }

    Slvs_hConstraint addSameOrientation(Slvs_hEntity n1, Slvs_hEntity n2,
                                  Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_SAME_ORIENTATION,0,0,0,0,n1,n2,group,h);
    }

    Slvs_hConstraint addAngle(double degree, bool supplement,
            Slvs_hEntity l1, Slvs_hEntity l2, Slvs_hEntity wrkpln=0, 
            Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        SLVS_INIT_CSTR(SLVS_C_ANGLE,wrkpln,degree,0,0,l1,l2);
        _var.other = supplement?1:0;
        return addConstraint(_var);
    }

    Slvs_hConstraint addPerpendicular(Slvs_hEntity l1, Slvs_hEntity l2,
            Slvs_hEntity wrkpln=0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_PERPENDICULAR,wrkpln,0,0,0,l1,l2,group,h);
    }

    Slvs_hConstraint addParallel(Slvs_hEntity l1, Slvs_hEntity l2,
             Slvs_hEntity wrkpln=0, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_PARALLEL,wrkpln,0,0,0,l1,l2,group,h);
    }

    Slvs_hConstraint addArcLineTangent(bool atEnd, Slvs_hEntity arc, 
            Slvs_hEntity line, Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        SLVS_INIT_CSTR(SLVS_C_ARC_LINE_TANGENT,0,0,0,0,arc,line);
        _var.other = atEnd?1:0;
        return addConstraint(_var);
    }

    Slvs_hConstraint addCubicLineTangent(bool atEnd, Slvs_hEntity cubic, 
                                Slvs_hEntity line, Slvs_hEntity wrkpln=0,
                                Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        SLVS_INIT_CSTR(SLVS_C_CUBIC_LINE_TANGENT,wrkpln,0,0,0,cubic,line);
        _var.other = atEnd?1:0;
        return addConstraint(_var);
    }

    Slvs_hConstraint addCurvesTangent(bool atEnd1,bool atEnd2, 
                Slvs_hEntity c1,Slvs_hEntity c2, Slvs_hEntity wrkpln, 
                Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        SLVS_INIT_CSTR(SLVS_C_CURVE_CURVE_TANGENT,wrkpln,0,0,0,c1,c2);
        _var.other = atEnd1?1:0;
        _var.other2 = atEnd2?1:0;
        return addConstraint(_var);
    }

    Slvs_hConstraint addEqualRadius(Slvs_hEntity c1, Slvs_hEntity c2,
                                    Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_EQUAL_RADIUS,0,0,0,0,c1,c2,group,h);
    }

    Slvs_hConstraint addWhereDragged(Slvs_hEntity pt, Slvs_hEntity wrkpln = 0,
                                     Slvs_hGroup group=0, Slvs_hConstraint h=0)
    {
        return addConstraintV(SLVS_C_WHERE_DRAGGED,wrkpln,0,pt,0,0,0,group,h);
    }
};

#endif  // defined _SLVS_SWIG_

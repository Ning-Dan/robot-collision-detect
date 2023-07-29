#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"

void test1(){
    std::shared_ptr<fcl::CollisionGeometry<double>> box1(new fcl::Box<double>(3,3,3));
    std::shared_ptr<fcl::CollisionGeometry<double>> box2(new fcl::Box<double>(1,1,1));

    fcl::Transform3d tf1 = fcl::Transform3d::Identity();
    fcl::CollisionObjectd obj1(box1,tf1);

    fcl::Transform3d tf2 = fcl::Transform3d::Identity();
    fcl::CollisionObjectd obj2(box2,tf2);

    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;

    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;//specify solver type with the default type is GST_LIBCCD

    fcl::collide(&obj1,&obj2,request,result);

    std::cout<<"test1 collide result:"<<result.isCollision()<<std::endl;
}

void test2(){
    std::shared_ptr<fcl::CollisionGeometry<double>> box1(new fcl::Box<double>(3,3,3));
    std::shared_ptr<fcl::CollisionGeometry<double>> box2(new fcl::Box<double>(1,1,1));

    fcl::Transform3d tf1 = fcl::Transform3d::Identity();
    fcl::CollisionObjectd obj1(box1,tf1);

    fcl::Transform3d tf2 = fcl::Transform3d::Identity();
    tf2.translation() = fcl::Vector3d{3,0,0};

    fcl::CollisionObjectd obj2(box2,tf2);

    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;

    fcl::collide(&obj1,&obj2,request,result);

    std::cout<<"test2 collide result:"<<result.isCollision()<<std::endl;
}

void test3(){
    std::shared_ptr<fcl::CollisionGeometry<double>> box1(new fcl::Box<double>(3,3,3));
    std::shared_ptr<fcl::CollisionGeometry<double>> box2(new fcl::Box<double>(1,1,1));

    fcl::Transform3d tf1 = fcl::Transform3d::Identity();
    fcl::CollisionObjectd obj1(box1,tf1);

    fcl::Transform3d tf2 = fcl::Transform3d::Identity();
    tf2.translation() = fcl::Vector3d{3,0,0};

    fcl::CollisionObjectd obj2(box2,tf2);

    fcl::CollisionRequestd request;
    fcl::CollisionResultd result;

    // fcl::collide(&obj1,&obj2,request,result);

    std::cout<<"test3 collide result:"<<result.isCollision()<<std::endl;

    fcl::DistanceRequestd dist_request(true);
    dist_request.distance_tolerance = 1e-4;
    fcl::DistanceResultd dist_result;

    fcl::distance(&obj1,&obj2,dist_request,dist_result);

    std::cout<<"test3 collide distance:"<<dist_result.min_distance<<std::endl;
    std::cout<<"test3 collide point 0:"<<dist_result.nearest_points[0]<<std::endl;
    std::cout<<"test3 collide point 1:"<<dist_result.nearest_points[1]<<std::endl;
    
}

int main(int argc,char **argv){
    test1();
    test2();
    test3();
}

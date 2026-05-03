#include "fingerlib/transformer.hpp"
#include "fingerlib/joint_trajectory.hpp"
#include "armadillo"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <numbers>

TEST_CASE("Basic usage of JointTrajectory class", "[JointTrajectory]")
{

    // Radius Matrix
    const double ra = 0.0025; // splay
    const double rb = 0.0025; // mcp
    const double rc = 0.0025; // pip/dip

    const arma::mat Ra = {{ra, 0, 0}, // splay
                          {0, rb, 0}, // mcp
                          {0, 0, rc}}; // pip/dip


    // Structure Matrix
    const double r11 = ra*3.5;
    const double r1 = 8 * 0.001;
    //const double r2 = 8 * 0.001;
    const double r3 = 4.5 * 0.001;
    //const double r4 = 4.5 * 0.001;
    const double r5 = 8 * 0.001;
    //const double r6 = 8 * 0.001;
    const double r7 = 4.5 * 0.001;
    //const double r8 = 4.5 * 0.001;
    const double r9 = 9 * 0.001;
    //const double r10 = 9 * 0.001;

    const arma::mat St = {{r11, -r3, -r1}, //splay joint 
                          {0, r7, r5}, //mcp joint
                          {0, 0, r9}}; // pip/dip joint

    // screw axes (x = joint, y = finger, z = up) (origin is on splay joint)
    const std::vector<arma::vec6> slist = {
        arma::vec6({0,0,1,0,0,0}),
        arma::vec6({-1,0,0,0,0,0.01776}),
        arma::vec6({-1,0,0,0,0,0.07776}),
        arma::vec6({-1,0,0,0,0,0.11836})
    };

    const arma::vec joint_min = {-0.2, -0.2, -0.01};
    const arma::vec joint_max = {0.2, 1.572, 1.572};

    // very simple from onshape
    const arma::mat44 M = {{1, 0, 0, 0},
                          {0, 1, 0, 0.16},
                          {0, 0, 1, 0},
                          {0, 0, 0, 1}};

    // 4 bar lengths
    const std::vector<double> four_bar_lengths = {
        8.83765 * 0.001,
        40.6 * 0.001,
        8.91536 * 0.001,
        37.79903 * 0.001,
    };


    // create transformer class
    auto transforms = Transformer{Ra, St, slist, M, four_bar_lengths, joint_min, joint_max};
    auto generator = JointTrajectory{transforms, 100, -0.25};

    SECTION("Sinusoidal Motion")
    {
        auto q_motor_list = generator.generate_sinusoid(1, 0.2, 1.0, 0.8);
        REQUIRE(q_motor_list.size() == 100);
        REQUIRE_THAT(q_motor_list[0](0), Catch::Matchers::WithinAbs(q_motor_list.back()(0), 1e-3));
    }

    SECTION("Linear Motion")
    {
        arma::vec start = {0, 0, 0};
        arma::vec end   = {0, 1, 0.8};

        const double v_max = 0.5;
        const double a_max = 1.0;
        const double dt    = 1.0 / 100.0;

        auto q_motor_list = generator.generate_linear(start, end, v_max, a_max);

        // Endpoints match in joint space
        REQUIRE_THAT(arma::norm(transforms.motor_to_joint(q_motor_list.front()) - start),
                     Catch::Matchers::WithinAbs(0.0, 1e-3));
        REQUIRE_THAT(arma::norm(transforms.motor_to_joint(q_motor_list.back()) - end),
                     Catch::Matchers::WithinAbs(0.0, 1e-3));

        for (uint i = 1; i < q_motor_list.size(); i++)
        {
            auto q_joint      = transforms.motor_to_joint(q_motor_list[i]);
            auto q_joint_prev = transforms.motor_to_joint(q_motor_list[i-1]);

            const double vel = arma::norm(q_joint - q_joint_prev) / dt;

            // std::cout << "step=" << i
            //           << "  q_joint=" << q_joint.t()
            //           << "  vel=" << vel << std::endl;

            // Monotonic progress toward end
            REQUIRE(arma::norm(q_joint - end) <= arma::norm(q_joint_prev - end) + 1e-6);

            // Velocity never exceeds v_max
            REQUIRE_THAT(vel, Catch::Matchers::WithinAbs(vel, v_max + 1e-3));
            REQUIRE(vel <= v_max + 1e-3);
        }
    }

    SECTION("Linear Motion - Ground Collision")
    {
        arma::vec start = {0, 0, 0};
        arma::vec end   = {0, 1, 0.8};

        const double v_max = 0.5;
        const double a_max = 1.0;

        auto generator_bad = JointTrajectory{transforms, 100, -0.1};

        REQUIRE_THROWS(generator_bad.generate_linear(start, end, v_max, a_max));  
    }

    SECTION("Cartesian Waypoints")
    {
        arma::vec start = {0, 0.15, -0.05};
        arma::vec end   = {0, 0.08, -0.1};  
        
        std::vector<arma::vec> waypoints = {start, end};

        auto q_motor_list = generator.generate_cartesian(waypoints, 1, 1);

        // for(auto q_motor: q_motor_list){
        //     std::cout << q_motor << std::endl;
        // }
    }
}
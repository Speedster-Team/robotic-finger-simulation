#include "fingerlib/transformer.hpp"
#include "armadillo"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <numbers>

TEST_CASE("Basic usage of Transformer class", "[Transformer]")
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

    const arma::vec joint_min = {-0.2, -0.2, 0};
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

    SECTION("Joint to motor space")
    {
        // initialize joint angle
        arma::vec q_joint = {.1,0,0};

        // calculate corresponding motor angles
        auto q_motor = transforms.joint_to_motor(q_joint);

        std::cout << "q_motor" << q_motor << std::endl;

        REQUIRE_THAT(q_motor(0), Catch::Matchers::WithinAbs(0.35, 1e-12));
        REQUIRE_THAT(q_motor(1), Catch::Matchers::WithinAbs(-.18, 1e-12));
        REQUIRE_THAT(q_motor(2), Catch::Matchers::WithinAbs(-0.32, 1e-12));
    }

    SECTION("Motor Space to Joint Space")
    {
        // initialize motor angle
        arma::vec q_motor = {0,0,3.6};

        // calculate corresponding motor angles
        auto q_joint = transforms.motor_to_joint(q_motor);

        std::cout << "q_joint" << q_joint << std::endl;

        REQUIRE_THAT(q_motor(0), Catch::Matchers::WithinAbs(0, 1e-12));
        REQUIRE_THAT(q_motor(1), Catch::Matchers::WithinAbs(0, 1e-12));
        //REQUIRE_THAT(q_motor(2), Catch::Matchers::WithinAbs(1, 1e-12));
    }

    // ground truths from sympy
    SECTION("4 Bar Linkage")
    {
        double pip_angle = 0.0;
        double dip_angle, speed_ratio;
        transforms.calculate_4bar_ratios(pip_angle, dip_angle, speed_ratio);

        REQUIRE_THAT(dip_angle, Catch::Matchers::WithinAbs(0.00182, 1e-1));
        REQUIRE_THAT(speed_ratio, Catch::Matchers::WithinAbs(0.922, 1e-1));

        pip_angle = 0.785;
        // double dip_angle, speed_ratio;
        transforms.calculate_4bar_ratios(pip_angle, dip_angle, speed_ratio);

        REQUIRE_THAT(dip_angle, Catch::Matchers::WithinAbs(0.785, 1e-1));
        REQUIRE_THAT(speed_ratio, Catch::Matchers::WithinAbs(0.981, 1e-1));

        pip_angle = 1.571;
        // double dip_angle, speed_ratio;
        transforms.calculate_4bar_ratios(pip_angle, dip_angle, speed_ratio);

        REQUIRE_THAT(dip_angle, Catch::Matchers::WithinAbs(1.571, 1e-1));
        REQUIRE_THAT(speed_ratio, Catch::Matchers::WithinAbs(1.205, 1e-1));

    }

    // from sympy, full test for this is if IK works
    SECTION("Coupled Jacobian")
    {
        arma::vec q_joint = {0,0,0};

        arma::mat J_truth = {
            {0.0,      -1.0,      -1.92766584},
            {0.0,      0,        0.0       },
            {1.0,      0.0,      0.0       },
            {0.0,      0.0,      0.0       },
            {0.0,      0.0,      0.0       },
            {0.0,      0.01776,  0.18755853}
        };

        auto J = transforms.get_jacobian_space(q_joint);


        REQUIRE_THAT(arma::abs(J - J_truth).max(), Catch::Matchers::WithinAbs(0.0, 1e-1));
    }

    SECTION("FK")
    {
        arma::vec q_joint = {0.1,0.5,1.5};

        auto q_ee = transforms.joint_to_end_effector(q_joint);

        auto ee_pos = q_ee.submat(0,3,2,3);

        std::cout << ee_pos << std::endl;

        auto q_joint_ik = transforms.end_effector_to_joint(ee_pos);

        std::cout << q_joint_ik << std::endl;


    }

}
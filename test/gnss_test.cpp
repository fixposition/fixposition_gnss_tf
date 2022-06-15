/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /  
 *   /  /\  \  
 *  /__/  \__\  Fixposition AG
 * 
 * @file gnss_test.cpp
 * @author Fixpositon AG (info@fixposition.com)
 * @brief 2021 Fixposition AG www.fixposition.com All rights reserved.
 * @date 2021-07-20
 * 
 */

/* SYSTEM / STL */
#include <math.h>

/* EXTERNAL */
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>

/* PACKAGE */
#include "fixposition_gnss_tf/gnss_tf.hpp"
#include "gtest/gtest.h"

namespace {

static const double ecef_err = 5e-4;
static const double rad_err = 1e-8;
static const std::string test_config_path = TEST_DIR + std::string("gnss_test.yaml");

/**
 * @brief Convert degrees to radians
 *
 * @tparam T
 * @param[in] degrees
 * @return constexpr T
 */
template <typename T>
constexpr inline T DegToRad(T degrees) {
    static_assert(::std::is_floating_point<T>::value, "Only floating point types allowed!");
    return degrees * M_PI / 180.0;
}

/**
 * @brief Convert radians to degrees
 *
 * @tparam T
 * @param[in] radians
 * @return constexpr T
 */
template <typename T>
constexpr inline T RadToDeg(T radians) {
    static_assert(::std::is_floating_point<T>::value, "Only floating point types allowed!");
    return radians * 180.0 / M_PI;
}

/**
 * @brief Convert llh from deg to rad, height component unchanged
 *
 * @param[in] llh_deg llh in degrees
 * @return Eigen::Vector3d llh in radian
 */
inline Eigen::Vector3d LlhDegToRad(const Eigen::Vector3d& llh_deg) {
    return Eigen::Vector3d(DegToRad(llh_deg.x()), DegToRad(llh_deg.y()), llh_deg.z());
}

/**
 * @brief Convert llh from rad to deg, height component unchanged
 *
 * @param[in] llh_rad llh in radian
 * @return Eigen::Vector3d llh in degrees
 */
inline Eigen::Vector3d LlhRadToDeg(const Eigen::Vector3d& llh_rad) {
    return Eigen::Vector3d(RadToDeg(llh_rad.x()), RadToDeg(llh_rad.y()), llh_rad.z());
}

void CompareEigenVec(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2, const double err = ecef_err) {
    EXPECT_EQ(vec1.size(), vec2.size());
    for (int i = 0; i < vec1.size(); ++i) {
        EXPECT_NEAR(vec1[i], vec2[i], err);
    }
}

void CompareLlh(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) {
    EXPECT_EQ(vec1.size(), vec2.size());
    EXPECT_NEAR(vec1.x(), vec2.x(), rad_err);
    EXPECT_NEAR(vec1.y(), vec2.y(), rad_err);
    EXPECT_NEAR(vec1.z(), vec2.z(), ecef_err);
}

class LlhEcefTest : public ::testing::Test {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Vector3d> llh_vec_;
    std::vector<Eigen::Vector3d> ecef_vec_;

    virtual void SetUp() final {
        YAML::Node LLH_ECEF = YAML::LoadFile(test_config_path)["LLH_ECEF"];
        for (auto elem : LLH_ECEF) {
            Eigen::Vector3d llh(elem["LLH"].as<std::vector<double>>().data());
            Eigen::Vector3d ecef(elem["ECEF"].as<std::vector<double>>().data());
            llh_vec_.push_back(llh);
            ecef_vec_.push_back(ecef);
        }
    }
};

TEST_F(LlhEcefTest, TestAllYaml) {
    const size_t num_tests = llh_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resllh = gnss_tf::TfWgs84LlhEcef(ecef);
        const Eigen::Vector3d resecef = gnss_tf::TfEcefWgs84Llh(llh);

        std::cout << "Res LLH  " << LlhRadToDeg(resllh).transpose().format(Eigen::IOFormat(15)) << std::endl;
        std::cout << "Res ECEF " << resecef.transpose().format(Eigen::IOFormat(15)) << std::endl;
        CompareLlh(llh, resllh);
        CompareEigenVec(ecef, resecef);
    }
}

class EnuEcefTest : public ::testing::Test {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Eigen::Vector3d> llh_vec_;
    std::vector<Eigen::Vector3d> enu_vec_;
    std::vector<Eigen::Vector3d> ecef_vec_;

    virtual void SetUp() final {
        YAML::Node ENU_ECEF = YAML::LoadFile(test_config_path)["ENU_ECEF"];
        for (auto elem : ENU_ECEF) {
            Eigen::Vector3d llh(elem["LLH"].as<std::vector<double>>().data());
            Eigen::Vector3d enu(elem["ENU"].as<std::vector<double>>().data());
            Eigen::Vector3d ecef(elem["ECEF"].as<std::vector<double>>().data());
            llh_vec_.push_back(llh);
            enu_vec_.push_back(enu);
            ecef_vec_.push_back(ecef);
        }
    }
};

TEST_F(EnuEcefTest, TfEnuEcef) {
    const size_t num_tests = llh_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d enu = enu_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resenu = gnss_tf::TfEnuEcef(ecef, llh);

        std::cout << "Res ENU  " << resenu.transpose().format(Eigen::IOFormat(15)) << std::endl;
        CompareEigenVec(enu, resenu);
    }
}

TEST_F(EnuEcefTest, TfEcefEnu) {
    const size_t num_tests = llh_vec_.size();
    for (size_t i = 0; i < num_tests; ++i) {
        const Eigen::Vector3d llh = LlhDegToRad(llh_vec_.at(i));
        const Eigen::Vector3d enu = enu_vec_.at(i);
        const Eigen::Vector3d ecef = ecef_vec_.at(i);

        const Eigen::Vector3d resecef = gnss_tf::TfEcefEnu(enu, llh);

        std::cout << "Res ECEF  " << resecef.transpose().format(Eigen::IOFormat(15)) << std::endl;
        CompareEigenVec(ecef, resecef);
    }
}

}  // anonymous namespace

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
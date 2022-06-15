/**
 *  @file
 *  @brief Declaration of Gnss coordinate transformation functions
 *
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /   Fixposition AG
 *   /  /\  \   All right reserved.
 *  /__/  \__\
 *
 */

#ifndef __GNSS_TRANSFORMATION_LIB_GNSS_TF_HPP__
#define __GNSS_TRANSFORMATION_LIB_GNSS_TF_HPP__

/* EXTERNAL */
#include <eigen3/Eigen/Core>

namespace gnss_tf {

/**
 * @brief Calculate the rotation matrix from ECEF to
 * ENU for a given reference latitude/longitude.
 * *
 * @param[in] lat Reference latitude [rad]
 * @param[in] lon Reference longitude [rad]
 * @return Eigen::Matrix3d Rotation matrix from ECEF -> ENU
 */
Eigen::Matrix3d RotEnuEcef(double lat, double lon);

/**
 * @brief Calculate the rotation matrix from ECEF to
 * ENU for a given reference origin.
 *
 * @param[in] in_pos Reference position in ECEF [m]
 * @return Eigen::Matrix3d Rotation matrix ECEF -> ENU
 */
Eigen::Matrix3d RotEnuEcef(const Eigen::Vector3d &ecef);

/**
 * @brief Transform ECEF coordinate to ENU with specified ENU-origin
 *
 * @param[in] xyz ECEF position to be transsformed [m]
 * @param[in] wgs84llh_ref ENU-origin in Geodetic coordinates (Lat[rad], Lon[rad], Height[m])
 * @return Eigen::Vector3d Position in ENU coordinates
 */
Eigen::Vector3d TfEnuEcef(const Eigen::Vector3d &ecef, const Eigen::Vector3d &wgs84llh_ref);

/**
 * @brief  Transform ENU coordinate to ECEF with specified ENU-origin
 *
 * @param[in] enu ENU position to be transsformed [m]
 * @param[in] wgs84llh_ref ENU-origin in Geodetic coordinates (Lat[rad], Lon[rad], Height[m])
 * @return Eigen::Vector3d
 */
Eigen::Vector3d TfEcefEnu(const Eigen::Vector3d &enu, const Eigen::Vector3d &wgs84llh_ref);

/**
 * @brief Convert Geodetic coordinates (latitude, longitude, height) to
 * ECEF (x, y, z).
 *
 * @param[in] wgs84llh Geodetic coordinates (Lat[rad], Lon[rad], Height[m])
 * @return Eigen::Vector3d ECEF coordinates [m]
 */
Eigen::Vector3d TfEcefWgs84Llh(const Eigen::Vector3d &wgs84llh);

/**
 * @brief Convert ECEF (x, y, z) coordinates to Lat Lon Height coordinates
 * (latitude, longitude, altitude).
 *
 * @param[in] ecef ECEF coordinates [m]
 * @return Eigen::Vector3d Geodetic coordinates (Lat[rad], Lon[rad], Height[m])
 */
Eigen::Vector3d TfWgs84LlhEcef(const Eigen::Vector3d &ecef);

}  // namespace gnss_tf

#endif
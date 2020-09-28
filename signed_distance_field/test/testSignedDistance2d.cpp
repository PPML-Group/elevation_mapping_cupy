//
// Created by rgrandia on 10.07.20.
//

#include <gtest/gtest.h>

#include "signed_distance_field/SignedDistance2d.h"
#include "signed_distance_field/PixelBorderDistance.h"

#include "naiveSignedDistance.h"

TEST(testSignedDistance2d, signedDistance2d_noObstacles) {
  const int n = 3;
  const int m = 4;
  const float resolution = 0.1;
  const grid_map::Matrix map = grid_map::Matrix::Ones(n, m);

  const auto occupancy = signed_distance_field::occupancyAtHeight(map, 2.0);
  const auto signedDistance = signed_distance_field::signedDistanceFromOccupancy(occupancy, resolution);

  ASSERT_TRUE((signedDistance.array() == signed_distance_field::INF).all());
}

TEST(testSignedDistance2d, signedDistance2d_allObstacles) {
  const int n = 3;
  const int m = 4;
  const float resolution = 0.1;
  const grid_map::Matrix map = grid_map::Matrix::Ones(n, m);

  const auto occupancy = signed_distance_field::occupancyAtHeight(map, 0.0);
  const auto signedDistance = signed_distance_field::signedDistanceFromOccupancy(occupancy, resolution);

  ASSERT_TRUE((signedDistance.array() == -signed_distance_field::INF).all());
}

TEST(testSignedDistance2d, signedDistance2d_mixed) {
  const int n = 2;
  const int m = 3;
  const float resolution = 1.0;
  grid_map::Matrix map(n, m);
  map << 0.0, 1.0, 1.0, 0.0, 0.0, 1.0;
  const auto occupancy = signed_distance_field::occupancyAtHeight(map, 0.5);

  const auto naiveSignedDistance = signed_distance_field::naiveSignedDistanceFromOccupancy(occupancy, resolution);
  const auto signedDistance = signed_distance_field::signedDistanceFromOccupancy(occupancy, resolution);
  ASSERT_TRUE(signed_distance_field::isEqualSdf(signedDistance, naiveSignedDistance, 1e-4));
}

TEST(testSignedDistance2d, signedDistance2d_oneObstacle) {
  const int n = 20;
  const int m = 30;
  const float resolution = 0.1;
  grid_map::Matrix map =  grid_map::Matrix::Zero(n, m);
  map(n/2, m/2) = 1.0;
  const auto occupancy = signed_distance_field::occupancyAtHeight(map, 0.5);

  const auto naiveSignedDistance = signed_distance_field::naiveSignedDistanceFromOccupancy(occupancy, resolution);
  const auto signedDistance = signed_distance_field::signedDistanceFromOccupancy(occupancy, resolution);
  ASSERT_TRUE(signed_distance_field::isEqualSdf(signedDistance, naiveSignedDistance, 1e-4));
}

TEST(testSignedDistance2d, signedDistance2d_oneFreeSpace) {
  const int n = 20;
  const int m = 30;
  const float resolution = 0.1;
  grid_map::Matrix map =  grid_map::Matrix::Ones(n, m);
  map(n/2, m/2) = 0.0;

  const auto occupancy = signed_distance_field::occupancyAtHeight(map, 0.5);

  const auto naiveSignedDistance = signed_distance_field::naiveSignedDistanceFromOccupancy(occupancy, resolution);
  const auto signedDistance = signed_distance_field::signedDistanceFromOccupancy(occupancy, resolution);
  ASSERT_TRUE(signed_distance_field::isEqualSdf(signedDistance, naiveSignedDistance, 1e-4));
}

TEST(testSignedDistance2d, signedDistance2d_debugcase) {
  const int n = 3;
  const int m = 3;
  const float resolution = 1.0;
  grid_map::Matrix map(n, m);
  map <<
  1.0, 1.0, 1.0,
  0.0, 1.0, 1.0,
  1.0, 1.0, 0.0;

  const auto occupancy = signed_distance_field::occupancyAtHeight(map, 0.5);

  const auto naiveSignedDistance = signed_distance_field::naiveSignedDistanceFromOccupancy(occupancy, resolution);
  const auto signedDistance = signed_distance_field::signedDistanceFromOccupancy(occupancy, resolution);
  ASSERT_TRUE(signed_distance_field::isEqualSdf(signedDistance, naiveSignedDistance, 1e-4));
}

TEST(testSignedDistance2d, signedDistance2d_random) {
  const int n = 20;
  const int m = 30;
  const float resolution = 1.0;
  grid_map::Matrix map =  grid_map::Matrix::Random(n, m); // random [-1.0, 1.0]

  // Check at different heights, resulting in different levels of sparsity.
  float heightStep = 0.1;
  for (float height = -1.0 - heightStep; height < 1.0 + heightStep; height+=heightStep) {
    const auto occupancy = signed_distance_field::occupancyAtHeight(map, height);

    const auto naiveSignedDistance = signed_distance_field::naiveSignedDistanceFromOccupancy(occupancy, resolution);
    const auto signedDistance = signed_distance_field::signedDistanceFromOccupancy(occupancy, resolution);
    ASSERT_TRUE(signed_distance_field::isEqualSdf(signedDistance, naiveSignedDistance, 1e-4)) << "height: " << height;
  }
}
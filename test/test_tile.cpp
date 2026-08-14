#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "tile.hpp"

using rviz_satellite::TileMapInformation;
using rviz_satellite::fromWGS;
using rviz_satellite::tileOffset;
using sensor_msgs::msg::NavSatFix;

static NavSatFix makeFix(double latitude, double longitude)
{
  NavSatFix fix;
  fix.latitude = latitude;
  fix.longitude = longitude;
  return fix;
}

/**
 * The tile index and the offset within that tile describe the same point, so their sum is the
 * continuous position in tile units and must not jump as a fix moves. A tile index derived by
 * truncation while the offset is derived by flooring breaks this wherever coordinates are
 * negative, which is the normal case west and north of a local map origin.
 */
static void expectContinuousAcrossSweep(
  const TileMapInformation & info, double from, double to, double step, bool sweep_latitude)
{
  double previous = 0.0;
  bool have_previous = false;
  for (double v = from; v <= to; v += step) {
    auto const fix = sweep_latitude ? makeFix(v, 0.0) : makeFix(0.0, v);
    auto const coord = fromWGS(fix, info);
    auto const offset = tileOffset(fix, info);
    double const position = sweep_latitude ? coord.y + offset.y : coord.x + offset.x;
    if (have_previous) {
      ASSERT_LT(std::abs(position - previous), 0.5)
        << "tile index and in-tile offset disagree at " << (sweep_latitude ? "latitude " : "longitude ")
        << v << ": position jumped from " << previous << " to " << position;
    }
    previous = position;
    have_previous = true;
  }
}

class LocalTileMap : public ::testing::Test
{
protected:
  TileMapInformation info;

  void SetUp() override
  {
    info.local_map = true;
    info.zoom = 0;
    // one tile spans 256 m, so a few thousandths of a degree cover several tiles
    info.meter_per_pixel_z0 = 1.0;
    // origin at the projected position of (0, 0), so a fix north or west of it is negative
    info.origin_x = 0.0;
    info.origin_y = 0.0;
    info.origin_crs = "EPSG:3857";
    info.transformation =
      proj_create_crs_to_crs(info.context, "EPSG:4326", info.origin_crs.c_str(), NULL);
    if (info.transformation == nullptr) {
      GTEST_SKIP() << "PROJ cannot create EPSG:4326 to EPSG:3857, is the PROJ database installed?";
    }
  }

  void TearDown() override
  {
    if (info.transformation != nullptr) {
      proj_destroy(info.transformation);
    }
  }
};

TEST_F(LocalTileMap, x_index_and_offset_agree_west_of_the_origin)
{
  expectContinuousAcrossSweep(info, -0.005, 0.005, 0.0002, false);
}

TEST_F(LocalTileMap, y_index_and_offset_agree_north_of_the_origin)
{
  expectContinuousAcrossSweep(info, -0.005, 0.005, 0.0002, true);
}

TEST_F(LocalTileMap, tiles_west_and_north_of_the_origin_are_negative)
{
  // roughly one tile west and north of the origin
  auto const coord = fromWGS(makeFix(0.0015, -0.0015), info);
  EXPECT_LT(coord.x, 0) << "a fix west of the origin must land on a negative tile column";
  EXPECT_LT(coord.y, 0) << "a fix north of the origin must land on a negative tile row";
}

TEST(GlobalTileMap, index_and_offset_agree)
{
  TileMapInformation info;
  info.local_map = false;
  info.zoom = 16;
  expectContinuousAcrossSweep(info, -0.02, 0.02, 0.0005, false);
  expectContinuousAcrossSweep(info, -0.02, 0.02, 0.0005, true);
}

TEST(GlobalTileMap, null_island_maps_to_the_south_east_quadrant)
{
  // pins the global projection: at zoom 1 the world is 2x2 tiles and (0, 0) is their common corner
  TileMapInformation info;
  info.local_map = false;
  info.zoom = 1;
  auto const coord = fromWGS(makeFix(0.0, 0.0), info);
  EXPECT_EQ(1, coord.x);
  EXPECT_EQ(1, coord.y);
  EXPECT_EQ(1, coord.z);
}

TEST(GlobalTileMap, coordinates_stay_within_the_zoom_grid)
{
  TileMapInformation info;
  info.local_map = false;
  info.zoom = 16;
  int const n = 1 << info.zoom;
  for (auto const & fix :
       {makeFix(85.0, -180.0), makeFix(-85.0, 180.0), makeFix(0.0, 0.0), makeFix(85.0, 180.0)}) {
    auto const coord = fromWGS(fix, info);
    EXPECT_GE(coord.x, 0);
    EXPECT_GE(coord.y, 0);
    EXPECT_LE(coord.x, n);
    EXPECT_LE(coord.y, n);
  }
}

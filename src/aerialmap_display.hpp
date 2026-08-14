/* Copyright 2014 Gareth Cross

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <string>
#include <utility>

#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"
#include "tile.hpp"
#include "tile_client.hpp"
#include "tile_object.hpp"

namespace rviz_satellite
{

/**
 * @brief Display a satellite map on the XY plane.
 */
class AerialMapDisplay : public rviz_common::RosTopicDisplay<sensor_msgs::msg::NavSatFix>
{
  Q_OBJECT

public:
  AerialMapDisplay();
  ~AerialMapDisplay() override;

  void onInitialize() override;
  void reset() override;
  void update(float, float) override;

protected Q_SLOTS:
  void updateAlpha();
  void updateDrawUnder();
  void updateTileUrl();
  void updateZoom();
  void updateBlocks();
  void updateLocalMap();
  void updateLocalTileMapInformation();

protected:
  void onEnable() override;
  void onDisable() override;

  void processMessage(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) override;

  bool validateMessage(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  bool validateProperties();

  bool shiftMap(TileCoordinate center_tile, Ogre::Vector2i offset, double size);

  void buildMap(TileCoordinate center_tile, double size);

  void buildTile(TileCoordinate coordinate, Ogre::Vector2i offset, double size);

  /// Whether a tile can exist on the configured tile server
  bool isTileInBounds(const TileCoordinate & coordinate) const;

  /// Drop all tiles and abort their requests; the caller must hold tiles_mutex_
  void clearTiles();

  void resetMap();

  /// Report either the missing tile count or an all clear on TILE_REQUEST_STATUS
  void updateTileRequestStatus();

  void resetTileServerError();

  /**
   * @brief React to a failed tile request
   *
   * Transient failures are retried after a backoff, so the display recovers on its own once the
   * connection is stable again. Permanent failures disable requests until a tile server relevant
   * property changes.
   */
  void handleTileRequestFailure(const tile_request_error & e);

  void updateAlpha(const rclcpp::Time & t);

  rclcpp::Duration tf_tolerance() const;

  double computeUTMrotation(double latitude, double longitude);

  rviz_common::properties::StringProperty * tile_url_property_ = nullptr;
  rviz_common::properties::IntProperty * zoom_property_ = nullptr;
  rviz_common::properties::IntProperty * blocks_property_ = nullptr;
  rviz_common::properties::FloatProperty * alpha_property_ = nullptr;
  rviz_common::properties::FloatProperty * timeout_property_ = nullptr;
  rviz_common::properties::FloatProperty * tf_tolerance_property_ = nullptr;
  rviz_common::properties::Property * draw_under_property_ = nullptr;

  rviz_common::properties::Property * local_map_property_ = nullptr;
  rviz_common::properties::FloatProperty * local_meter_per_pixel_z0_property_ = nullptr;
  rviz_common::properties::StringProperty * local_origin_crs_property_ = nullptr;
  rviz_common::properties::FloatProperty * local_origin_x_property_ = nullptr;
  rviz_common::properties::FloatProperty * local_origin_y_property_ = nullptr;

  rviz_common::properties::TfFrameProperty * orientation_frame_property_ = nullptr;
  rviz_common::properties::BoolProperty * use_local_tiles_property_ = nullptr;
  rviz_common::properties::BoolProperty * visualize_in_utm_frame = nullptr;

  std::mutex tiles_mutex_;
  TileClient tile_client_;

  TileMapInformation tile_map_info_;
  std::map<TileId, std::future<QImage>> pending_tiles_;
  std::map<TileId, TileObject> tiles_;
  /// Coordinate the tile field is centered on; empty while no map is built
  std::optional<TileCoordinate> center_tile_;

  sensor_msgs::msg::NavSatFix::ConstSharedPtr last_fix_;
  /// set on permanent failures only; cleared when a tile server relevant property changes
  bool tile_server_had_errors_{false};
  /// no tiles are requested before this point in time, to back off after transient failures
  std::chrono::steady_clock::time_point retry_tiles_after_{};
  int consecutive_tile_failures_{0};
  /// tiles of the current map the tile server reported as not available
  int missing_tiles_{0};

  /// Delay before the map is rebuilt after a transient failure; doubled per consecutive failure
  static constexpr int TILE_RETRY_BASE_DELAY_MS = 1000;
  /// Caps the backoff at TILE_RETRY_BASE_DELAY_MS << (MAX_BACKOFF_EXPONENT - 1)
  static constexpr int MAX_BACKOFF_EXPONENT = 4;

  static const QString MESSAGE_STATUS;
  static const QString TILE_REQUEST_STATUS;
  static const QString PROPERTIES_STATUS;
  static const QString ORIENTATION_STATUS;
  static const QString TRANSFORM_STATUS;
  static const QString PROJ_TRANSFORM_STATUS;
};

}  // namespace rviz_satellite

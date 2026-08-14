/* Copyright 2018-2019 TomTom N.V.

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

#include <QImage>
#include <QtNetwork>
#include <cstdint>
#include <exception>
#include <future>
#include <map>
#include <string>

#include "tile.hpp"

namespace rviz_satellite
{

class tile_request_error : public std::exception
{
private:
  std::string message_;
  bool transient_;

public:
  explicit tile_request_error(const std::string & message, bool transient = false)
  : message_(message), transient_(transient)
  {
  }

  const char * what() const noexcept override { return message_.c_str(); }

  /**
   * @brief Whether the failure is expected to resolve itself
   *
   * Transient failures (timeouts, dropped connections, truncated images, server side errors)
   * are worth retrying. Permanent failures (unknown host path, missing tile, denied access)
   * indicate a misconfiguration and require a property change.
   */
  bool transient() const noexcept { return transient_; }
};

/**
 * @brief Download tiles from a Tile server.
 */
class TileClient : public QObject
{
  Q_OBJECT

private:
  struct PendingRequest
  {
    std::promise<QImage> promise;
    /// in-flight reply, or nullptr while waiting for a scheduled retry
    QNetworkReply * reply = nullptr;
    int attempts = 0;
    /// distinguishes this request from a later request for the same tile
    uint64_t generation = 0;
  };

  /// Number of times a single tile is requested before its promise fails
  static constexpr int MAX_ATTEMPTS = 4;
  /// Delay before the first retry; doubled for every further attempt
  static constexpr int RETRY_BASE_DELAY_MS = 250;

  QNetworkAccessManager * manager_;
  std::map<TileId, PendingRequest> requests_;
  uint64_t generation_counter_ = 0;

  QNetworkRequest make_request(const TileId & tile_id) const;
  void start_request(const TileId & tile_id, PendingRequest & pending);
  void schedule_retry(const TileId & tile_id, uint64_t generation, int delay_ms);

public:
  TileClient();
  ~TileClient() override;

  /**
   * @brief Load a specific tile
   *
   * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached. Otherwise they
   * get downloaded.
   *
   * If server url contains "file://", local filesystem will be used.
   *
   */
  std::future<QImage> request(const TileId & tile_id);

  /**
   * @brief Load a specific tile from filesystem
   *
   */
  std::future<QImage> request_local(const TileId & tile_id);

  /**
   * @brief Load a specific tile from internet
   *
   * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached. Otherwise they
   * get downloaded.
   *
   * If a request for the same tile is still in flight, it is aborted and superseded by the new
   * one. The future of the superseded request reports a broken promise.
   */
  std::future<QImage> request_remote(const TileId & tile_id);

  /**
   * @brief Abort all in-flight requests and drop their promises
   *
   * Required whenever the caller discards the corresponding futures, so no network traffic
   * outlives the tiles it was requested for, and so the same tiles can be requested again.
   */
  void abort_all();

private Q_SLOTS:
  void request_finished(QNetworkReply * reply);
};

}  // namespace rviz_satellite

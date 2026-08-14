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

#include "tile_client.hpp"

#include <QImage>
#include <QImageReader>
#include <QStandardPaths>
#include <QString>
#include <QTimer>
#include <QtCore>
#include <QtNetwork>
#include <chrono>
#include <regex>
#include <utility>

#include "rviz_common/logging.hpp"

namespace rviz_satellite
{

/**
 * @brief Whether a network error is expected to resolve itself on a retry
 *
 * Covers the failure modes of a briefly unstable connection. Everything else (missing tile,
 * denied access, malformed url) is treated as permanent, so a misconfigured tile server is
 * not hammered with retries.
 */
static bool is_transient(QNetworkReply::NetworkError error)
{
  switch (error) {
    case QNetworkReply::ConnectionRefusedError:
    case QNetworkReply::RemoteHostClosedError:
    case QNetworkReply::HostNotFoundError:
    case QNetworkReply::TimeoutError:
    case QNetworkReply::TemporaryNetworkFailureError:
    case QNetworkReply::NetworkSessionFailedError:
    case QNetworkReply::ProxyConnectionRefusedError:
    case QNetworkReply::ProxyConnectionClosedError:
    case QNetworkReply::ProxyNotFoundError:
    case QNetworkReply::ProxyTimeoutError:
    case QNetworkReply::InternalServerError:
    case QNetworkReply::ServiceUnavailableError:
    case QNetworkReply::UnknownNetworkError:
    case QNetworkReply::UnknownProxyError:
    case QNetworkReply::UnknownServerError:
      return true;
    default:
      return false;
  }
}

TileClient::TileClient() : manager_(new QNetworkAccessManager(this)), requests_()
{
  connect(manager_, SIGNAL(finished(QNetworkReply *)), SLOT(request_finished(QNetworkReply *)));
  QNetworkDiskCache * disk_cache = new QNetworkDiskCache(this);
  QString const cache_path =
    QDir(QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation))
      .filePath("rviz_satellite");
  disk_cache->setCacheDirectory(cache_path);
  manager_->setCache(disk_cache);
}

TileClient::~TileClient() { abort_all(); }

/**
 * @brief Request a specific tile
 *
 * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached.
 * Otherwise they are fetched from the tile server given in the @p tile_id.
 */
std::future<QImage> TileClient::request(TileId const & tile_id)
{
  if (tile_id.server_url.find("file://") != std::string::npos) {
    return request_local(tile_id);
  } else {
    return request_remote(tile_id);
  }
}

QNetworkRequest TileClient::make_request(TileId const & tile_id) const
{
  // see https://foundation.wikimedia.org/wiki/Maps_Terms_of_Use#Using_maps_in_third-party_services
  auto const request_url = QUrl(QString::fromStdString(tileURL(tile_id)));
  QNetworkRequest request(request_url);
  char constexpr agent[] =
    "rviz_satellite " RVIZ_SATELLITE_VERSION " (https://github.com/Kettenhoax/rviz_satellite)";
  request.setHeader(QNetworkRequest::KnownHeaders::UserAgentHeader, agent);
  QVariant variant;
  variant.setValue(tile_id);
  request.setAttribute(
    QNetworkRequest::CacheLoadControlAttribute, QNetworkRequest::CacheLoadControl::PreferCache);
  request.setAttribute(QNetworkRequest::User, variant);
  return request;
}

void TileClient::start_request(TileId const & tile_id, PendingRequest & pending)
{
  auto const request = make_request(tile_id);
  ++pending.attempts;
  RVIZ_COMMON_LOG_DEBUG_STREAM(
    "Requesting tile " << request.url().toString().toStdString() << " (attempt "
                       << pending.attempts << ")");
  pending.reply = manager_->get(request);
}

void TileClient::schedule_retry(TileId const & tile_id, uint64_t generation, int delay_ms)
{
  QTimer::singleShot(delay_ms, this, [this, tile_id, generation]() {
    auto it = requests_.find(tile_id);
    if (it == requests_.end() || it->second.generation != generation || it->second.reply) {
      // the request was aborted or superseded while the retry was pending
      return;
    }
    start_request(tile_id, it->second);
  });
}

std::future<QImage> TileClient::request_remote(TileId const & tile_id)
{
  // A request for this tile may still be in flight, because the map is rebuilt whenever the
  // zoom, the url or the center tile change, without waiting for outstanding replies. Supersede
  // the stale request instead of refusing the new one; refusing would leave the tile permanently
  // unrequestable, since nothing ever retracts the stale entry.
  auto existing = requests_.find(tile_id);
  if (existing != requests_.end()) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Superseding in-flight request for tile '" << tile_id << "'");
    PendingRequest superseded = std::move(existing->second);
    // erase before aborting: abort() re-enters request_finished, which must not find this entry
    requests_.erase(existing);
    if (superseded.reply) {
      superseded.reply->abort();
      superseded.reply->deleteLater();
    }
    // superseded.promise is broken here, its future reports std::future_error
  }

  auto & pending = requests_[tile_id];
  pending.generation = ++generation_counter_;
  auto future = pending.promise.get_future();
  start_request(tile_id, pending);
  return future;
}

std::future<QImage> TileClient::request_local(TileId const & tile_id)
{
  std::future<QImage> f = std::async(std::launch::async, [tile_id] {
    auto const filename_uri = tileURL(tile_id);

    auto filename = std::regex_replace(filename_uri, std::regex("file://"), "");

    QImageReader reader(QString::fromStdString(filename));

    if (!reader.canRead()) {
      RVIZ_COMMON_LOG_DEBUG_STREAM("Unable to decode image at " << filename);
      return QImage{};
    }

    auto image = reader.read().mirrored();

    if (image.isNull()) {
      RVIZ_COMMON_LOG_DEBUG_STREAM("QImageReader able to decode but read failed for " << filename);
    }

    return image;
  });

  return f;
}

void TileClient::abort_all()
{
  // move out first: abort() re-enters request_finished, which must not find these entries
  auto aborted = std::move(requests_);
  requests_.clear();
  for (auto & entry : aborted) {
    if (entry.second.reply) {
      entry.second.reply->abort();
      entry.second.reply->deleteLater();
    }
  }
  // the promises are broken here, their futures report std::future_error
}

void TileClient::request_finished(QNetworkReply * reply)
{
  // the reply is still readable until the event loop runs again
  reply->deleteLater();

  const QVariant variant = reply->request().attribute(QNetworkRequest::User);
  auto tile_id = variant.value<TileId>();

  auto promise_it = requests_.find(tile_id);
  if (promise_it == requests_.end() || promise_it->second.reply != reply) {
    // the request was aborted or superseded, there is no promise left to fulfil
    return;
  }
  // from here on, every path either erases the entry or schedules a retry for it; leaving a
  // satisfied promise behind would make this tile unrequestable for the rest of the session
  promise_it->second.reply = nullptr;

  const QUrl url = reply->url();
  auto const error = reply->error();
  if (error != QNetworkReply::NoError) {
    if (is_transient(error) && promise_it->second.attempts < MAX_ATTEMPTS) {
      int const delay_ms = RETRY_BASE_DELAY_MS << (promise_it->second.attempts - 1);
      RVIZ_COMMON_LOG_WARNING_STREAM(
        "Tile request for " << url.toString().toStdString() << " failed ("
                            << reply->errorString().toStdString() << "), retrying in " << delay_ms
                            << " ms");
      schedule_retry(tile_id, promise_it->second.generation, delay_ms);
      return;
    }
    promise_it->second.promise.set_exception(std::make_exception_ptr(
      tile_request_error(reply->errorString().toStdString(), is_transient(error))));
    requests_.erase(promise_it);
    return;
  }

  // log if tile comes from cache or web
  bool const from_cache = reply->attribute(QNetworkRequest::SourceIsFromCacheAttribute).toBool();
  if (from_cache) {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Loaded tile from cache " << url.toString().toStdString());
  } else {
    RVIZ_COMMON_LOG_DEBUG_STREAM("Loaded tile from web " << url.toString().toStdString());
  }

  QImageReader reader(reply);
  if (!reader.canRead()) {
    // a truncated or empty body is a symptom of an unstable connection, so retry as well
    if (promise_it->second.attempts < MAX_ATTEMPTS) {
      int const delay_ms = RETRY_BASE_DELAY_MS << (promise_it->second.attempts - 1);
      RVIZ_COMMON_LOG_WARNING_STREAM(
        "Failed to decode image at " << url.toString().toStdString() << ", retrying in "
                                     << delay_ms << " ms");
      schedule_retry(tile_id, promise_it->second.generation, delay_ms);
      return;
    }
    promise_it->second.promise.set_exception(
      std::make_exception_ptr(tile_request_error("Failed to decode tile image", true)));
    requests_.erase(promise_it);
    RVIZ_COMMON_LOG_ERROR_STREAM(
      "Failed to decode image at " << reply->request().url().toString().toStdString());
    return;
  }
  promise_it->second.promise.set_value(reader.read().mirrored());
  requests_.erase(promise_it);
}

}  // namespace rviz_satellite
